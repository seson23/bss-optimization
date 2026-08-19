# 전기차 배터리 교환 스테이션(BSS) 운영 최적화

SoH(State of Health, 배터리 성능 상태) 등급별 고객 수요를 반영해, 전기차 배터리 교환 스테이션(BSS) 네트워크의 초기재고·운송스케줄을 통합 최적화하는 연구입니다. Gurobi 기반 MILP(혼합정수계획법)로 모델링했고, 2025년 대한산업공학회 춘계학술대회에서 포스터로 발표했습니다.

## 시스템 개요

![BSS 운영 구조](images/system_overview.png)

고객은 BSS를 방문해 사용하던 방전 배터리를 반납하고, 원하는 최소 SoH 수준을 지정해 완충 배터리를 요청합니다. 요청 수준보다 높은 등급을 제공하면 서비스 품질 향상에 따른 추가 이익이 발생하고, 반대로 해당 등급 이상의 배터리가 부족해 교환하지 못하면 패널티가 부과됩니다.

각 BSS는 반납된 방전 배터리 중 일부를 자체 충전 설비로 완충 배터리로 전환하고, 충전되지 않은 배터리는 정기 운송을 통해 중앙 BCS(Battery Charging Station)로 이송합니다. BCS는 다수의 BSS를 총괄하는 단일 거점으로, 저속·고효율 충전 방식을 사용해 배터리를 충전한 뒤 다시 BSS 네트워크로 공급합니다. 운송 차량은 사전에 정해진 스케줄에 따라 하루 두 번(수거 트립·공급 트립) 담당 BSS를 순회하며 BCS-BSS 간 배터리 교환을 수행합니다.

## 모델 정식화 (MILP)

### 집합 (Sets)

* $S = \{1,\dots,NS\}$ — BSS 집합
* $H = \{1,\dots,NH\}$ — 배터리 SoH 등급 집합
* $C = S \cup \{0\}$ — 충전소 집합 (BSS + BCS)
* $T = \{1,\dots,NT\}$ — 시간대 집합
* $R_p$ — 수거 트립(pick-up trip) 집합
* $R_d$ — 공급 트립(delivery trip) 집합
* $R = R_p \cup R_d$ — 전체 운송 트립 집합
* $K = \{1,\dots,NK\}$ — 운송 트럭 집합
* $S_k = \{1,\dots,NS_k\}$ — 트럭 $k$가 담당하는 BSS 집합

### 파라미터 (Parameters)

* $\alpha_j$ — SoH 등급 $j$ 배터리 수요 미충족 패널티
* $\beta_{j,h}$ — 등급 $j$ 요청 고객에게 등급 $h$ 제공 시 발생하는 이익
* $DM_{s,i,j,t}$ — BSS $s$에서 시간 $t$에 등급 $i$를 반납하고 등급 $j$를 요청하는 수요량
* $NC_c$ — 충전소 $c$의 충전기 수
* $CT_c$ — 충전소 $c$에서 배터리 1개 충전 소요 시간
* $EP_t$ — 시간 $t$의 전력 요금
* $ET_c$ — 충전소 $c$에서 배터리 1개당 단위 시간 전력 소모량
* $TCP$ — 트럭 최대 운송 용량
* $T_r^{str}$ — 트립 $r$의 운송 시작 시간
* $T_r^{end}$ — 트립 $r$의 운송 종료 시간
* $TA^{dis}_{s,t}$ — BSS $s$에서 시간 $t$에 수거 가능 여부 (1 또는 0)
* $TA^{chg}_{s,t}$ — BSS $s$에서 시간 $t$에 공급 가능 여부 (1 또는 0)
* $TI$ — 전체 BSS·SoH 등급 초기 재고 총합

### 변수 (Variables)

* $n^{fc}_{c,h,t}$ — 충전소 $c$, 시간 $t$의 등급 $h$ 완충 배터리 재고
* $n^{dc}_{c,h,t}$ — 충전소 $c$, 시간 $t$의 등급 $h$ 방전 배터리 재고
* $cst_{c,h,t}$ — 충전소 $c$, 시간 $t$에 충전 시작한 등급 $h$ 배터리 수
* $dc^{fc}_{c,h,t}$ — 충전소 $c$, 시간 $t$에 충전 완료된 등급 $h$ 배터리 수
* $gb_{s,i,j,h,t}$ — 등급 $i$ 반납·등급 $j$ 요청·등급 $h$ 제공으로 충족된 수요량
* $sq_{s,i,j,t}$ — BSS $s$, 시간 $t$의 등급 $i \to j$ 수요 미충족량
* $ec_t$ — 시간 $t$의 총 전력 소비량
* $l^{dis}_{s,h,t}$ — BSS $s$, 시간 $t$에 트럭이 수거한 등급 $h$ 배터리 수
* $l^{chg}_{s,h,t}$ — BSS $s$, 시간 $t$에 트럭이 공급한 등급 $h$ 배터리 수
* $col_{h,t}$ — BCS에서 시간 $t$에 트럭으로부터 수거한 등급 $h$ 배터리 수
* $shp_{h,t}$ — BCS에서 시간 $t$에 트럭에 공급한 등급 $h$ 배터리 수

### 목적함수

전체 운영 비용(전력 사용 + 수요 미충족 패널티 − 상향 공급 이익)을 최소화합니다.

$$
\begin{aligned}
\min \quad & \sum_{t \in T} EP_t \cdot ec_t + \sum_{s \in S}\sum_{i \in H}\sum_{j \in H}\sum_{t \in T} \alpha_j \cdot sq_{s,i,j,t} \\
& - \sum_{s \in S}\sum_{i \in H}\sum_{h \in H}\sum_{j=h+1}^{|H|}\sum_{t \in T} \beta_{j,h} \cdot gb_{s,i,j,h,t}
\end{aligned}
$$

### 제약식

**재고 밸런스 (2)~(5)** — BSS·BCS 각각의 방전/완충 배터리 재고가 반납·충전·수거·공급에 따라 갱신됩니다.

$$
\begin{aligned}
n^{dc}_{s,h,t} &= n^{dc}_{s,h,t-1} + rb_{s,h,t} - dc^{fc}_{s,h,t} - l^{dis}_{s,h,t} \qquad \forall s \in S, h \in H, t \in T \\
n^{fc}_{s,h,t} &= n^{fc}_{s,h,t-1} + l^{chg}_{s,h,t} + dc^{fc}_{s,h,t} - \sum_{i \in H}\sum_{j=h}^{|H|} gb_{s,i,j,h,t} \qquad \forall s \in S, h \in H, t \in T \\
n^{dc}_{0,h,t} &= n^{dc}_{0,h,t-1} + col_{h,t} - dc^{fc}_{0,h,t} \qquad \forall h \in H, t \in T \\
n^{fc}_{0,h,t} &= n^{fc}_{0,h,t-1} + dc^{fc}_{0,h,t} - shp_{h,t} \qquad \forall h \in H, t \in T
\end{aligned}
$$

**수요 밸런스 (6)~(7)** — 요청 수요는 제공량과 미충족량의 합으로 구성되고, 반납량과 제공량이 일치합니다.

$$
\begin{aligned}
DM_{s,i,j,t} &= \sum_{h=1}^{j} gb_{s,i,j,h,t} + sq_{s,i,j,t} \qquad \forall s \in S,\ i,j \in H,\ t \in T \\
rb_{s,i,t} &= \sum_{j \in H}\sum_{h \in H} gb_{s,i,j,h,t} \qquad \forall s \in S,\ i \in H,\ t \in T
\end{aligned}
$$

**충전 제약 (8)~(12)** — 충전 시작 후 일정 시간 뒤 완충 전환, 충전기 용량 제한, 전력 소비량 산정입니다.

$$
\begin{aligned}
dc^{fc}_{c,h,(t+CT_c)} &= cst_{c,h,t} \qquad \forall c \in C,\ h \in H \\
\sum_{h \in H}\sum_{i=t-CT_c+1}^{t} cst_{c,h,i} &\leq NC_c \qquad \forall c \in C,\ t \geq CT_c - 1 \\
\sum_{h \in H}\sum_{i=0}^{t} cst_{c,h,i} &\leq NC_c \qquad \forall c \in C,\ t < CT_c - 1 \\
ec_t &= \sum_{c \in C}\sum_{h \in H}\sum_{i=t-CT_c+1}^{t} ET_c \cdot cst_{c,h,i} \qquad t \geq CT_c - 1 \\
ec_t &= \sum_{c \in C}\sum_{h \in H}\sum_{i=0}^{t} ET_c \cdot cst_{c,h,i} \qquad t < CT_c - 1
\end{aligned}
$$

**운송 제약 (13)~(18)** — 트립별 수거·공급 총량 일치, 트럭 적재 용량 제한, 트립 가능 시간대 제한입니다.

$$
\begin{aligned}
\sum_{k \in K}\sum_{s \in S_k}\sum_{t \in T_r} l^{dis}_{s,h,t} &= col_{h,T_r^{end}} \qquad \forall r \in R_p,\ h \in H \\
\sum_{k \in K}\sum_{s \in S_k}\sum_{t \in T_r} l^{chg}_{s,h,t} &= shp_{h,T_r^{str}} \qquad \forall r \in R_d,\ h \in H \\
\sum_{s \in S_k}\sum_{h \in H}\sum_{t \in T_r} l^{dis}_{s,h,t} &\leq TCP \qquad \forall k \in K,\ r \in R_p \\
\sum_{s \in S_k}\sum_{h \in H}\sum_{t \in T_r} l^{chg}_{s,h,t} &\leq TCP \qquad \forall k \in K,\ r \in R_d \\
\sum_{h \in H} l^{dis}_{s,h,t} &\leq TCP \cdot TA^{dis}_{s,t} \qquad \forall s \in S,\ t \in T \\
\sum_{h \in H} l^{chg}_{s,h,t} &\leq TCP \cdot TA^{chg}_{s,t} \qquad \forall s \in S,\ t \in T
\end{aligned}
$$

**초기값/종료값 제약 (19)~(24)** — 초기 재고 총합 고정, BCS 초기 재고는 총합의 일정 비율, 종료 재고는 초기 재고의 일정 범위 내로 제한(일별 연속성 확보)합니다.

$$
\begin{aligned}
\sum_{s \in S}\sum_{h \in H} n^{fc}_{s,h,0} &= TI \\
\sum_{s \in S}\sum_{h \in H} n^{dc}_{s,h,0} &= TI \\
n^{fc}_{0,h,0} &= \phi \cdot TI \qquad \forall h \in H \\
n^{dc}_{0,h,0} &= \phi \cdot TI \qquad \forall h \in H \\
(1-\epsilon)\, n^{fc}_{c,h,0} &\leq n^{fc}_{c,h,|T|} \leq (1+\epsilon)\, n^{fc}_{c,h,0} \qquad \forall c \in C \\
(1-\epsilon)\, n^{dc}_{c,h,0} &\leq n^{dc}_{c,h,|T|} \leq (1+\epsilon)\, n^{dc}_{c,h,0} \qquad \forall c \in C
\end{aligned}
$$

## 실험 및 결과

### 실험 1 — 초기 재고 배분 전략 비교

BSS 수(6/12/18)별로 최적화 배분, 균등 분배, 수요 비율 배분 세 가지 초기 재고 전략을 비교했습니다.

| BSS 수 | 목적함수 값 (×10⁴) — 최적화 | 균등 분배 | 수요 비율 |
|---|---|---|---|
| 6 | 174.9 | 205.0 | 201.1 |
| 12 | 393.9 | 410.5 | 400.5 |
| 18 | 635.9 | 719.9 | 704.4 |
| 평균 | 352.5 | 392.7 | 384.2 |

최적화 배분이 균등 분배 대비 평균 **5.8%** 낮은 운영비용을 기록했습니다.

### 실험 2 — 운송 시간대 전략 비교

Baseline(최적화 모델) 대비 수요 대응 모델, 재고 대응 모델 두 가지 운송 시점 전략을 비교했습니다.

| BSS 수 | Baseline | 수요 대응 | 재고 대응 |
|---|---|---|---|
| 6 | 174.9 | 200.0 | 175.8 |
| 12 | 393.9 | 444.1 | 394.3 |
| 18 | 635.9 | 712.6 | 637.0 |
| 평균 | 352.5 | 398.5 | 353.5 |

운송 시간대 설정에 따라 목적함수 값이 최대 18%, 패널티 비용이 최대 18.5%까지 차이가 났으며, 적절한 수거·공급 시점 설계가 운영 효율성에 직접적인 영향을 미침을 확인했습니다.

## 코드 구조

```
├── model/
│   └── bss_model.py      # Gurobi 기반 MILP 모델 (집합·파라미터·변수·목적함수·제약식·결과 시각화)
└── images/
    └── system_overview.png
```

수식은 이 README에 LaTeX으로 직접 작성되어 있습니다 (GitHub 마크다운의 KaTeX 렌더링 사용).

실제 실험에서는 서울시 동작구 전기차 충전소 이용 데이터를 기반으로 파라미터를 구성했으며, 이 저장소의 `bss_model.py`는 모델 구조를 보여주기 위해 샘플 데이터로 대체한 버전입니다.

## 사용 스택

Python, Gurobi, pandas, matplotlib
