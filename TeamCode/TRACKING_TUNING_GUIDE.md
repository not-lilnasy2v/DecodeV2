# Ghid de Tunare Tracking V6 - Turela Lock-On

## Cum functioneaza algoritmul

```
Odometrie (pozitie robot) → calculeaza unghi catre goal → odomAngle
Limelight (AprilTag tx) → corecteaza cu llOffset (exponential smoothing)
Heading rate → feedforward pentru rotatie (filtrat cu HR_FILTER)
Translational velocity → feedforward pentru miscare laterala
targetAngle = odomAngle + llOffset + llIntegral*LL_KI + TURELA_OFFSET_DEG + feedforward
smoothedTarget = filtrare adaptiva (T_ALPHA / STATIONARY_ALPHA / disturbance 0.95)
PD controller: power = KP * error + KD * filteredDerivative
Derivative filter: filteredDeriv = 0.8 * prevFiltered + 0.2 * rawDeriv (elimina tremur)
Deadzone: opreste servoul cand eroare < DEADZONE SI viteza < 50°/s
Boundary protection: frana automata la 15° de limite
```

## Parametrii si ce fac

### PD Controller (cei mai importanti)

| Parametru | Valoare | Ce face |
|-----------|---------|---------|
| `SERVO_KP` | 0.0030 | Forta de corectie. Mai mare = mai rapid dar risc oscilatie |
| `SERVO_KD` | 0.0005 | Amortizare. Mai mare = opreste oscilatia |
| `DERIV_FILTER` | 0.80 | Filtreaza zgomotul derivatei. 0.0=nefiltrat, 0.95=foarte smooth |
| `TURELA_DEADZONE` | 2.0° | Zona moarta. Mai mare = mai stabil, mai putin precis |

### Limelight Fusion

| Parametru | Valoare | Ce face |
|-----------|---------|---------|
| `ALPHA` | 0.30 | Cat de repede adopta corectia LL. 0.1=lent, 0.5=rapid |
| `DECAY` | 0.997 | Cat tine minte corectia cand pierde tag-ul. 0.99=uita repede, 0.999=tine mult |
| `LL_KI` | 0.03 | Integral - corecteaza erori persistente mici |
| `LL_MAX_INTEGRAL` | 15.0 | Limita integrala |

### Feedforward

| Parametru | Valoare | Ce face |
|-----------|---------|---------|
| `GAIN_DEG` | 1.0 | Compenseaza rotatia robotului. Mai mare = anticipeaza mai mult |
| `TRANS_FF_GAIN` | 0.12 | Compenseaza miscarea laterala |
| `VEL_LEAD_TIME` | 0.15 | Predictie pozitie viitoare. Mai mare = mai predictiv |
| `HR_FILTER` | 0.30 | Cat de rapid filtreaza heading rate |

### Smoothing

| Parametru | Valoare | Ce face |
|-----------|---------|---------|
| `T_ALPHA` | 0.50 | Viteza de urmarire in miscare |
| `STATIONARY_ALPHA` | 0.40 | Viteza de urmarire stationar (mai mic = mai stabil) |
| `DISTURBANCE_THRESHOLD` | 17.0° | Eroare peste care reactia e maxima (0.95) |

### Offset

| Parametru | Valoare | Ce face |
|-----------|---------|---------|
| `TURELA_OFFSET_DEG` | -15.0 (Rosu) / -25.0 (Albastru) | Offset mecanic per parte |

## Procedura de tunare cu TrackingTuner V6

### Porneste "Tracking Tuner V6" din Driver Station

Deschide FTC Dashboard (192.168.43.1:8080/dash) pentru a modifica constantele LIVE.

### Pas 1: Elimina oscilatia (STATIONAR)

1. Pune robotul nemiscat fata de goal, apasa X = tracking ON
2. Uita-te la telemetrie: "Eroare" si "Power"
3. **Daca turela tremura/oscileaza**:
   - Creste `SERVO_KD` cu +0.0001 (0.0005 → 0.0006)
   - Daca tot oscileaza: creste `DERIV_FILTER` (0.80 → 0.85)
   - Daca tot oscileaza: scade `SERVO_KP` cu -0.0005 (0.003 → 0.0025)
   - Ultimul resort: creste `TURELA_DEADZONE` (2.0 → 2.5)
4. **Target**: Eroarea < 2° si Power = 0 (in deadzone)

### Pas 2: Verifica precizia

1. Tot stationar, turela pe goal
2. Telemetrie: "Eroare" trebuie sa fie < 3°
3. **Daca eroarea e constant mare** (ex: mereu +10°):
   - Ajusteaza `TURELA_OFFSET_DEG` cu ±2-3° pana cand artefactele ajung in goal
4. **Daca LL offset creste mult** (>20):
   - Odometria e imprecisa - recalibreaza pinpoint-ul
5. **Daca eroarea fluctueaza** fara miscare:
   - Scade `ALPHA` (0.30 → 0.20) - LL influenteaza prea mult
   - Sau scade `LL_KI` (0.03 → 0.02) - integrala acumuleaza prea mult

### Pas 3: Test in miscare

1. Mergi cu robotul stanga-dreapta, turela trebuie sa urmareasca goal-ul
2. **Daca ramane in urma** (lag vizibil):
   - Creste `T_ALPHA` cu +0.05 (0.50 → 0.55)
   - Sau creste `GAIN_DEG` (1.0 → 1.2) daca lag-ul e la rotire
3. **Daca suprareactioneaza** (overshoot):
   - Scade `T_ALPHA` (0.50 → 0.45)
   - Scade `GAIN_DEG` (1.0 → 0.8)
   - Scade `VEL_LEAD_TIME` (0.15 → 0.10)
4. **Daca face overshoot la miscare laterala**:
   - Scade `TRANS_FF_GAIN` (0.12 → 0.08)

### Pas 4: Test cu lovituri/impact

1. Loveste usor robotul, turela trebuie sa revina rapid
2. **Daca revine prea incet**:
   - Creste `SERVO_KP` sau scade `TURELA_DEADZONE`
   - `DISTURBANCE_THRESHOLD` la 15.0 (mai sensibil la perturbatii)
3. **Daca supracompenseaza**:
   - Creste `SERVO_KD`

### Pas 5: Test Limelight

1. Acopera camera 3s, descopera
2. **Daca turela sare la descoperire**:
   - Scade `ALPHA` (0.30 → 0.20)
3. **Daca drifteaza mult cand e acoperit**:
   - Creste `DECAY` (0.997 → 0.999) - nu peste 0.999

### Pas 6: Copiaza valorile

Cand esti multumit, copiaza constantele din Dashboard in:
- `Tel.java` (Albastru TeleOp) - cu OFFSET = -25.0
- `TelR.java` (Rosu TeleOp) - cu OFFSET = -15.0
- `sistemeAuto.java` (Autonome) - recomandare: aceleasi valori

## Valori curente

```
=== PD Controller ===
KP=0.0030  KD=0.0005  DERIV_FILTER=0.80  DEADZONE=2.0°

=== LL Fusion ===
ALPHA=0.30  DECAY=0.997  LL_KI=0.03  MAX_INTEGRAL=15.0

=== Feedforward ===
GAIN_DEG=1.0  TRANS_FF=0.12  VEL_LEAD=0.15  HR_FILTER=0.30

=== Smoothing ===
T_ALPHA=0.50  STATIONARY_ALPHA=0.40  DISTURBANCE=17.0°

=== Offset ===
Rosu (TelR + FirstAuto + RosuAuto): -15.0
Albastru (Tel + AproapeAlbastru):   -25.0

=== sistemeAuto (diferente fata de TeleOp) ===
DEADZONE=1.5  ALPHA=0.20  KD=0.0004  GAIN_DEG=0.8
HR_FILTER=0.25  T_ALPHA=0.55  LL_KI=0.02  TRANS_FF=0.10
STATIONARY_ALPHA=0.50  MAX_INTEGRAL=10.0
```

## Regula de aur

**Schimba UN SINGUR parametru** → testeaza → apoi urmatorul.
Daca ceva merge prost, revino la valoarea anterioara.
