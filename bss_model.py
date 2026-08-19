"""
전기차 배터리 교환 스테이션(BSS) 운영 최적화 모델
SoH(배터리 성능 상태) 등급별 고객 수요를 반영한 초기재고·운송스케줄 통합 최적화 (Gurobi MILP)

수식 정의는 docs/formulation.md 참고.
"""

from itertools import product

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from gurobipy import GRB, Model, quicksum

# =========================================================
# 1. 집합 (Sets)
# =========================================================
T = range(0, 48)          # 시간대
H = [1, 2, 3]              # SoH 등급
S = [1, 2, 3, 4]           # BSS
C = S + [0]                 # 충전소 전체 (BSS + BCS, BCS는 0번)
K = [1, 2]                  # 운송 트럭

R_p = ["p1", "p2"]          # 수거 트립
R_d = ["d1", "d2"]          # 공급 트립
R = R_p + R_d                # 전체 운송 트립

# 트럭별 담당 BSS
S_k = {
    1: [1, 2],
    2: [3, 4],
}

# 트립별 운행 시간대
T_r = {
    "p1": range(6, 10),
    "d1": range(10, 14),
    "p2": range(30, 34),
    "d2": range(34, 38),
}
T_str = {r: T_r[r][0] for r in R}
T_end = {r: T_r[r][-1] for r in R}

# =========================================================
# 2. 파라미터 (Parameters)
# =========================================================
EP = {t: 3 if 36 <= t <= 44 else 2 for t in T}   # 시간대별 전력 요금
NC = {s: 15 for s in S}                            # 충전소별 충전기 수
NC[0] = 30
ET = 5     # 배터리당 단위 시간 전력 소모량
CT = 2     # 충전 소요 시간
TCP = 20   # 트럭 최대 운송 용량

alpha = 50  # 수요 미충족 패널티 계수
beta = {(h, i): 10 * (i - h) for h in H for i in H if h <= i}  # 상향 공급 이익 계수

# 반납량 RB / 요청 수요량 DM (실제 운영 시 서울시 충전소 이용 데이터 기반, 여기서는 샘플 데이터로 대체)
np.random.seed(30)
RB = {(s, h, t): np.random.randint(3 + (h - 1), 5 + (h - 1)) for s in S for h in H for t in T if t != 0}
DM = {(s, h, t): np.random.randint(4 + (h - 1), 6 + (h - 1)) for s in S for h in H for t in T if t != 0}

# 트립 스케줄에 따른 BSS별 수거/공급 가능 시점
TA_dis = {(s, t): 0 for s in S for t in T}
TA_chg = {(s, t): 0 for s in S for t in T}

for k in K:
    for r in R_p:
        for i, s in enumerate(S_k[k]):
            for t in T_r[r]:
                if t == T_r[r][i + 1]:
                    TA_dis[s, t] = 1

for k in K:
    for r in R_d:
        for i, s in enumerate(S_k[k]):
            for t in T_r[r]:
                if t == T_r[r][i + 1]:
                    TA_chg[s, t] = 1

# =========================================================
# 3. 모델 및 변수 (Model & Variables)
# =========================================================
m = Model("BSS_Optimization")

n_fc = m.addVars(C, H, T, lb=0, vtype=GRB.INTEGER, name="n_fc")   # 완충 배터리 재고
n_dc = m.addVars(C, H, T, lb=0, vtype=GRB.INTEGER, name="n_dc")   # 방전 배터리 재고
dc_fc = m.addVars(C, H, T, lb=0, vtype=GRB.INTEGER, name="dc_fc")  # 충전 완료 수량
gb = m.addVars(S, H, H, T, lb=0, vtype=GRB.INTEGER, name="gb")     # 등급별 제공 수량
ec = m.addVars(T, lb=0, vtype=GRB.CONTINUOUS, name="ec")             # 전력 소비량
sq = m.addVars(S, H, T, lb=0, vtype=GRB.INTEGER, name="sq")         # 수요 미충족량
l_dis = m.addVars(S, H, T, lb=0, vtype=GRB.INTEGER, name="l_dis")   # BSS 수거량
l_chg = m.addVars(S, H, T, lb=0, vtype=GRB.INTEGER, name="l_chg")   # BSS 공급량
cst = m.addVars(C, H, T, lb=0, vtype=GRB.INTEGER, name="cst")       # 충전 시작 수량
col = m.addVars(H, T, lb=0, vtype=GRB.INTEGER, name="col")           # BCS 수거량
shp = m.addVars(H, T, lb=0, vtype=GRB.INTEGER, name="shp")           # BCS 공급량

# =========================================================
# 4. 목적함수: 전력비용 + 수요 미충족 패널티 - 상향 공급 이익 최소화  (식 1)
# =========================================================
m.setObjective(
    quicksum(EP[t] * ec[t] for t in T[1:])
    + alpha * quicksum(sq[s, h, t] for s in S for h in H for t in T[1:])
    + quicksum(beta[h, i] * gb[s, h, i, t] for s in S for h in H for i in H for t in T[1:] if h <= i),
    GRB.MINIMIZE,
)

# =========================================================
# 5. 제약식 (Constraints)
# =========================================================

# 재고 밸런스 (식 2~5)
for s in S:
    for h in H:
        for t in T[1:]:
            m.addConstr(n_dc[s, h, t] == n_dc[s, h, t - 1] + RB[s, h, t] - dc_fc[s, h, t] - l_dis[s, h, t],
                        name="BSS_DC_Balance")
            m.addConstr(n_fc[s, h, t] == n_fc[s, h, t - 1] + l_chg[s, h, t] + dc_fc[s, h, t]
                        - quicksum(gb[s, h, i, t] for i in H if h <= i), name="BSS_FC_Balance")

for h in H:
    for t in T[1:]:
        m.addConstr(n_dc[0, h, t] == n_dc[0, h, t - 1] + col[h, t] - dc_fc[0, h, t], name="CS_DC_Balance")
        m.addConstr(n_fc[0, h, t] == n_fc[0, h, t - 1] - shp[h, t] + dc_fc[0, h, t], name="CS_FC_Balance")

# 수요 밸런스 (식 6)
for s in S:
    for h in H:
        for t in T[1:]:
            m.addConstr(DM[s, h, t] == quicksum(gb[s, i, h, t] for i in H if i <= h) + sq[s, h, t],
                        name="Demand_Balance")

# 충전 관련 제약 (식 8~12)
for c in C:
    for h in H:
        for t in T:
            if t - CT >= 0:
                m.addConstr(dc_fc[c, h, t] == cst[c, h, t - CT], name="Charging_Time")
            else:
                m.addConstr(dc_fc[c, h, t] == 0, name="Charging_Time")

for c in C:
    for t in T:
        if t - CT + 1 >= 0:
            m.addConstr(quicksum(cst[c, h, i] for h in H for i in range(t - CT + 1, t + 1)) <= NC[c],
                        name="Charge_Capacity")
        else:
            m.addConstr(quicksum(cst[c, h, t] for h in H) <= NC[c], name="Charge_Capacity")

for t in T:
    if t - CT + 1 >= 0:
        m.addConstr(ec[t] == ET * quicksum(cst[c, h, i] for c in C for h in H for i in range(t - CT + 1, t + 1)),
                    name="Power_Consumption")
    else:
        m.addConstr(ec[t] == ET * quicksum(cst[c, h, t] for c in C for h in H), name="Power_Consumption")

# 운송 트립 제약 (식 13~18)
for r in R_p:
    for h in H:
        m.addConstr(
            quicksum(l_dis[s, h, T_r[r][i + 1]] for k in K for i, s in enumerate(S_k[k])) == col[h, T_end[r]],
            name="Collected_Total")

for r in R_d:
    for h in H:
        m.addConstr(
            quicksum(l_chg[s, h, T_r[r][i + 1]] for k in K for i, s in enumerate(S_k[k])) == shp[h, T_str[r]],
            name="Supplied_Total")

for s in S:
    for t in T:
        m.addConstr(quicksum(l_dis[s, h, t] for h in H) <= TCP * TA_dis[s, t], name="Collection_Availability")
        m.addConstr(quicksum(l_chg[s, h, t] for h in H) <= TCP * TA_chg[s, t], name="Supply_Availability")

for k in K:
    for r in R_p:
        m.addConstr(
            quicksum(l_dis[s, h, t] for i, s in enumerate(S_k[k]) for h in H for t in T_r[r] if t == T_r[r][i + 1])
            <= TCP, name="Collection_Capacity")
    for r in R_d:
        m.addConstr(
            quicksum(l_chg[s, h, t] for i, s in enumerate(S_k[k]) for h in H for t in T_r[r] if t == T_r[r][i + 1])
            <= TCP, name="Supply_Capacity")

# 초기값 및 등급 제약 (식 19~24)
for c in C:
    for h in H:
        m.addConstr(n_fc[c, h, 0] == 10, name="Initial_FC")
        m.addConstr(n_dc[c, h, 0] == 10, name="Initial_DC")

# 고객이 요청한 등급보다 낮은 등급은 제공 불가
for s in S:
    for h in H:
        for i in H:
            for t in T:
                if h > i:
                    m.addConstr(gb[s, h, i, t] == 0, name="Grade_Restriction")

for h in H:
    for t in T:
        if not any(t == T_end[r] for r in R_p):
            m.addConstr(col[h, t] == 0, name="Collection_Timing")
        if not any(t == T_str[r] for r in R_d):
            m.addConstr(shp[h, t] == 0, name="Supply_Timing")

# =========================================================
# 6. 최적화 실행
# =========================================================
m.optimize()

if m.status == GRB.INFEASIBLE:
    print("모델이 infeasible합니다.")
    m.computeIIS()
    m.write("model_infeasible.ilp")

if m.status == GRB.OPTIMAL:
    print(f"총 목적함수 값: {m.ObjVal}")

    # -----------------------------------------------------
    # 7. 결과 시각화
    # -----------------------------------------------------
    ec_vals = [ec[t].X for t in T]
    plt.figure(figsize=(10, 5))
    plt.plot(T, ec_vals, marker="o")
    plt.title("Energy Consumption Over Time")
    plt.xlabel("Time")
    plt.ylabel("Energy Consumption")
    plt.grid(True)
    plt.show()

    sq_total = [sum(sq[s, h, t].X for s in S for h in H) for t in T]
    plt.figure(figsize=(10, 5))
    plt.plot(T, sq_total, marker="o", color="r")
    plt.title("Total Shortage Quantity Over Time")
    plt.xlabel("Time")
    plt.ylabel("Shortage Quantity")
    plt.grid(True)
    plt.show()

    fc_bss1 = pd.DataFrame({f"SoH {h}": [n_fc[1, h, t].X for t in T] for h in H}, index=T)
    fc_bss1.plot(figsize=(12, 6), marker="o", title="Fully Charged Batteries at BSS 1 Over Time")
    plt.xlabel("Time")
    plt.ylabel("Battery Quantity")
    plt.grid(True)
    plt.show()

    dc_cs = pd.DataFrame({f"SoH {h}": [n_dc[0, h, t].X for t in T] for h in H}, index=T)
    dc_cs.plot(figsize=(12, 6), marker="o", title="Discharged Batteries at BCS Over Time")
    plt.xlabel("Time")
    plt.ylabel("Battery Quantity")
    plt.grid(True)
    plt.show()

    fc_cs = pd.DataFrame({f"SoH {h}": [n_fc[0, h, t].X for t in T] for h in H}, index=T)
    fc_cs.plot(figsize=(12, 6), marker="o", title="Fully Charged Batteries at BCS Over Time")
    plt.xlabel("Time")
    plt.ylabel("Battery Quantity")
    plt.grid(True)
    plt.show()

    gb_sum_per_t = [
        sum(gb[s, i, h, t].X for s in S for i in H for h in H) for t in T
    ]
    plt.figure(figsize=(12, 6))
    plt.plot(T, gb_sum_per_t, marker="o", color="seagreen")
    plt.title("Total Grade-Upgrade Supply Quantity Over Time")
    plt.xlabel("Time")
    plt.ylabel("Quantity")
    plt.grid(True)
    plt.show()

else:
    print("Optimal solution not found.")
