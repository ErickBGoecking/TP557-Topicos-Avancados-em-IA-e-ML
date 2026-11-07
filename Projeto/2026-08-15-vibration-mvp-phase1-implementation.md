# Vibration MVP Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Montar e validar uma bancada ESP32 DevKit + MPU6050 + motor/cooler que colete vibrações a 100 Hz, gere CSVs rotulados C0/C1 e produza métricas básicas para decidir se os sinais são separáveis.

**Architecture:** O ESP32 lê o MPU6050 via I2C e transmite linhas CSV pela Serial USB. Um script Python no computador registra sessões independentes em arquivos CSV; outro script valida taxa de amostragem e calcula magnitude, média, RMS, desvio padrão e pico-a-pico por janelas de 1 s. Nesta fase não há inferência embarcada, FFT, TFLite Micro, MQTT ou dashboard.

**Tech Stack:** ESP32 Arduino Core, `Wire`, `Adafruit_MPU6050`, `Adafruit_Sensor`, Python 3.11+, `pyserial`, `pandas`, `numpy`, `matplotlib`, `pytest`.

## Global Constraints

- ESP32 DevKit V1 (ou equivalente com I2C nos GPIO 21/22).
- MPU6050/GY-521 alimentado em 3V3; GND comum apenas entre ESP32 e sensor.
- Motor/cooler alimentado por fonte separada adequada; nunca pelo pino 3V3/5V do ESP32.
- SDA = GPIO 21; SCL = GPIO 22; endereço esperado do MPU6050 = `0x68`.
- Aquisição alvo = 100 Hz, período nominal = 10.000 us.
- Campos mínimos no dataset: `timestamp_ms,ax_g,ay_g,az_g,label,session_id`.
- C0 = fixação nominal e velocidade estável.
- C1 = alteração segura e reprodutível de rigidez/fixação ou segunda velocidade fixa; nunca usar massa solta em parte girante.
- Descartar os 2 s iniciais de cada sessão; coletar pelo menos 60 s por sessão; fazer 3 sessões independentes por condição.
- Fase 1 não inclui FFT, autoencoder, quantização INT8, TensorFlow Lite Micro, Wi-Fi, MQTT ou dashboard.

---

## File map

- `firmware/01_i2c_scanner/01_i2c_scanner.ino`: detecta dispositivos I2C e confirma `0x68`.
- `firmware/02_mpu6050_static_test/02_mpu6050_static_test.ino`: valida leitura estática e ordem de grandeza de 1 g.
- `firmware/03_vibration_stream_100hz/03_vibration_stream_100hz.ino`: transmite `timestamp_ms,ax_g,ay_g,az_g` a 100 Hz.
- `host/serial_capture.py`: captura uma sessão via USB e acrescenta `label` e `session_id`.
- `host/analysis.py`: valida frequência e calcula métricas por janelas de 1 s.
- `host/requirements.txt`: dependências Python.
- `tests/test_analysis.py`: testes unitários das métricas e da estimativa de frequência.
- `data/raw/`: saída dos CSVs de coleta.

### Task 1: Validar barramento I2C e endereço do sensor

**Files:**
- Create: `firmware/01_i2c_scanner/01_i2c_scanner.ino`

**Interfaces:**
- Consumes: ESP32 com SDA GPIO 21 e SCL GPIO 22.
- Produces: saída Serial contendo `FOUND,0x68` quando o MPU6050 estiver acessível.

- [ ] **Step 1: Fazer a montagem elétrica sem o motor ligado**

Ligações:

```text
ESP32 DevKit       GY-521 / MPU6050
-----------        ----------------
3V3      ---------- VCC
GND      ---------- GND
GPIO 21  ---------- SDA
GPIO 22  ---------- SCL

Motor/cooler: fonte separada; manter desligado neste teste.
```

- [ ] **Step 2: Criar o scanner I2C**

```cpp
#include <Wire.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Serial.println("I2C_SCAN_START");
}

void loop() {
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("FOUND,0x%02X\n", address);
      ++found;
    }
  }
  Serial.printf("I2C_SCAN_END,count=%u\n", found);
  delay(3000);
}
```

- [ ] **Step 3: Compilar e gravar no ESP32**

Usar Arduino IDE com placa compatível com o ESP32 DevKit e Serial Monitor em `115200 baud`.

Resultado esperado:

```text
I2C_SCAN_START
FOUND,0x68
I2C_SCAN_END,count=1
```

Se aparecer `0x69`, verificar o estado de AD0. Se nenhum endereço aparecer, revisar VCC/GND/SDA/SCL antes de seguir.

- [ ] **Step 4: Registrar a evidência do teste**

Critério de aprovação: `0x68` aparece em 10 varreduras consecutivas sem desconexão.

---

### Task 2: Validar leitura estática e escala do MPU6050

**Files:**
- Create: `firmware/02_mpu6050_static_test/02_mpu6050_static_test.ino`

**Interfaces:**
- Consumes: MPU6050 acessível em `0x68`.
- Produces: leituras em g e norma `|a|` próxima de 1 g com a bancada parada.

- [ ] **Step 1: Instalar bibliotecas Arduino**

Instalar pelo Library Manager:

```text
Adafruit MPU6050
Adafruit Unified Sensor
```

- [ ] **Step 2: Criar teste estático**

```cpp
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;
constexpr float MS2_TO_G = 1.0f / 9.80665f;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("ERROR,MPU6050_NOT_FOUND");
    while (true) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(200);

  Serial.println("ax_g,ay_g,az_g,magnitude_g");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  const float ax = a.acceleration.x * MS2_TO_G;
  const float ay = a.acceleration.y * MS2_TO_G;
  const float az = a.acceleration.z * MS2_TO_G;
  const float mag = sqrtf(ax * ax + ay * ay + az * az);

  Serial.printf("%.5f,%.5f,%.5f,%.5f\n", ax, ay, az, mag);
  delay(100);
}
```

- [ ] **Step 3: Executar com sensor rigidamente apoiado e motor desligado**

Critério de aprovação: a média da coluna `magnitude_g` durante 10 s deve ficar aproximadamente entre `0.90 g` e `1.10 g`. Um eixo tende a concentrar a gravidade dependendo da orientação física.

- [ ] **Step 4: Testar orientação**

Girar o conjunto 90 graus e confirmar que a componente próxima de `1 g` migra para outro eixo sem alterar drasticamente a magnitude total.

---

### Task 3: Implementar streaming estável a 100 Hz

**Files:**
- Create: `firmware/03_vibration_stream_100hz/03_vibration_stream_100hz.ino`

**Interfaces:**
- Consumes: MPU6050 validado.
- Produces: linhas Serial `timestamp_ms,ax_g,ay_g,az_g` aproximadamente a cada 10 ms.

- [ ] **Step 1: Criar firmware de aquisição temporizada**

```cpp
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

constexpr uint32_t SAMPLE_PERIOD_US = 10000;  // 100 Hz
constexpr float MS2_TO_G = 1.0f / 9.80665f;
uint32_t nextSampleUs = 0;

void setup() {
  Serial.begin(230400);
  delay(1000);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("#ERROR,MPU6050_NOT_FOUND");
    while (true) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(200);

  Serial.println("timestamp_ms,ax_g,ay_g,az_g");
  nextSampleUs = micros();
}

void loop() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextSampleUs) < 0) {
    return;
  }

  nextSampleUs += SAMPLE_PERIOD_US;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  const float ax = a.acceleration.x * MS2_TO_G;
  const float ay = a.acceleration.y * MS2_TO_G;
  const float az = a.acceleration.z * MS2_TO_G;

  Serial.printf("%lu,%.6f,%.6f,%.6f\n",
                static_cast<unsigned long>(millis()), ax, ay, az);
}
```

- [ ] **Step 2: Executar primeiro com motor desligado**

Critério de aprovação: nenhuma linha malformada e sem resets do ESP32 durante 60 s.

- [ ] **Step 3: Ligar o motor/cooler por fonte separada**

Fixar o MPU6050 rigidamente na base/carcaça estacionária, nunca em parte rotativa. Manter cabos longe do rotor.

Critério de aprovação: o ESP32 continua transmitindo sem resets, desconexões ou valores travados.

---

### Task 4: Criar captura de sessão em CSV no computador

**Files:**
- Create: `host/serial_capture.py`
- Create: `host/requirements.txt`
- Create directory: `data/raw/`

**Interfaces:**
- Consumes: Serial com quatro colunas do firmware da Task 3.
- Produces: CSV com `timestamp_ms,ax_g,ay_g,az_g,label,session_id`.

- [ ] **Step 1: Criar dependências**

`host/requirements.txt`:

```text
pyserial>=3.5,<4
pandas>=2.2,<3
numpy>=2.0,<3
matplotlib>=3.9,<4
pytest>=8,<9
```

- [ ] **Step 2: Criar capturador**

```python
from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import serial


def parse_sample(line: str) -> tuple[int, float, float, float] | None:
    if not line or line.startswith("#") or line.startswith("timestamp_ms"):
        return None
    parts = line.split(",")
    if len(parts) != 4:
        return None
    try:
        return int(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
    except ValueError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--label", choices=["C0", "C1"], required=True)
    parser.add_argument("--session", required=True)
    parser.add_argument("--seconds", type=float, default=62.0)
    parser.add_argument("--discard-seconds", type=float, default=2.0)
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--output-dir", default="data/raw")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"{args.label}_{args.session}.csv"

    with serial.Serial(args.port, args.baud, timeout=1) as ser, output_path.open(
        "w", newline="", encoding="utf-8"
    ) as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_ms", "ax_g", "ay_g", "az_g", "label", "session_id"])

        ser.reset_input_buffer()
        start = time.monotonic()
        saved = 0
        malformed = 0

        while time.monotonic() - start < args.seconds:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            sample = parse_sample(raw)
            if sample is None:
                if raw and not raw.startswith("#") and not raw.startswith("timestamp_ms"):
                    malformed += 1
                continue

            elapsed = time.monotonic() - start
            if elapsed < args.discard_seconds:
                continue

            writer.writerow([*sample, args.label, args.session])
            saved += 1

    print(f"saved={saved} malformed={malformed} file={output_path}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Instalar dependências em ambiente virtual**

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r host/requirements.txt
```

No Windows PowerShell, ativar com `.venv\Scripts\Activate.ps1`.

- [ ] **Step 4: Fazer uma coleta de ensaio de 12 s**

Linux/macOS, exemplo:

```bash
python host/serial_capture.py --port /dev/ttyUSB0 --label C0 --session test01 --seconds 12
```

Windows, exemplo:

```powershell
python host/serial_capture.py --port COM5 --label C0 --session test01 --seconds 12
```

Critério de aprovação: aproximadamente 1.000 linhas salvas após o descarte de 2 s e `malformed=0` ou número próximo de zero.

---

### Task 5: Implementar métricas e testes automatizados

**Files:**
- Create: `host/analysis.py`
- Create: `tests/test_analysis.py`

**Interfaces:**
- Consumes: DataFrame com `timestamp_ms,ax_g,ay_g,az_g`.
- Produces: `estimate_sampling_hz(df) -> float` e `window_features(df, window_size=100) -> DataFrame`.

- [ ] **Step 1: Escrever testes que falham**

`tests/test_analysis.py`:

```python
import numpy as np
import pandas as pd

from host.analysis import estimate_sampling_hz, window_features


def make_constant_dataset(n: int = 300) -> pd.DataFrame:
    return pd.DataFrame(
        {
            "timestamp_ms": np.arange(n) * 10,
            "ax_g": np.zeros(n),
            "ay_g": np.zeros(n),
            "az_g": np.ones(n),
            "label": ["C0"] * n,
            "session_id": ["s1"] * n,
        }
    )


def test_estimate_sampling_hz_for_10ms_intervals():
    df = make_constant_dataset()
    assert abs(estimate_sampling_hz(df) - 100.0) < 0.01


def test_window_features_constant_1g_signal():
    df = make_constant_dataset()
    features = window_features(df, window_size=100)
    assert len(features) == 3
    assert np.allclose(features["magnitude_mean_g"], 1.0)
    assert np.allclose(features["magnitude_rms_g"], 1.0)
    assert np.allclose(features["magnitude_std_g"], 0.0)
    assert np.allclose(features["magnitude_ptp_g"], 0.0)
```

- [ ] **Step 2: Rodar testes e verificar falha**

```bash
pytest tests/test_analysis.py -v
```

Esperado: falha de importação porque `host.analysis` ainda não existe.

- [ ] **Step 3: Implementar análise mínima**

`host/analysis.py`:

```python
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd


def estimate_sampling_hz(df: pd.DataFrame) -> float:
    timestamps = df["timestamp_ms"].to_numpy(dtype=float)
    if len(timestamps) < 2:
        raise ValueError("at least two samples are required")
    dt_ms = np.diff(timestamps)
    positive = dt_ms[dt_ms > 0]
    if len(positive) == 0:
        raise ValueError("timestamps must increase")
    return 1000.0 / float(np.median(positive))


def window_features(df: pd.DataFrame, window_size: int = 100) -> pd.DataFrame:
    if window_size <= 0:
        raise ValueError("window_size must be positive")

    work = df.copy()
    work["magnitude_g"] = np.sqrt(
        work["ax_g"] ** 2 + work["ay_g"] ** 2 + work["az_g"] ** 2
    )

    records: list[dict[str, float | int | str]] = []
    full_windows = len(work) // window_size

    for index in range(full_windows):
        chunk = work.iloc[index * window_size : (index + 1) * window_size]
        magnitude = chunk["magnitude_g"].to_numpy(dtype=float)
        records.append(
            {
                "window": index,
                "label": str(chunk["label"].iloc[0]) if "label" in chunk else "",
                "session_id": str(chunk["session_id"].iloc[0]) if "session_id" in chunk else "",
                "magnitude_mean_g": float(np.mean(magnitude)),
                "magnitude_rms_g": float(np.sqrt(np.mean(magnitude**2))),
                "magnitude_std_g": float(np.std(magnitude)),
                "magnitude_ptp_g": float(np.ptp(magnitude)),
            }
        )

    return pd.DataFrame.from_records(records)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--window-size", type=int, default=100)
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    hz = estimate_sampling_hz(df)
    features = window_features(df, args.window_size)

    print(f"samples={len(df)}")
    print(f"sampling_hz={hz:.2f}")
    print(features.describe(include="all").to_string())


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Rodar testes e verificar aprovação**

```bash
pytest tests/test_analysis.py -v
```

Esperado: `2 passed`.

---

### Task 6: Executar protocolo experimental C0/C1

**Files:**
- Create through capture: `data/raw/C0_s01.csv`, `C0_s02.csv`, `C0_s03.csv`, `C1_s01.csv`, `C1_s02.csv`, `C1_s03.csv`.

**Interfaces:**
- Consumes: bancada estável e `serial_capture.py`.
- Produces: seis sessões independentes de aproximadamente 60 s úteis cada.

- [ ] **Step 1: Fixar o sensor**

O MPU6050 deve ficar rigidamente preso à parte estacionária da carcaça/base do motor. Marcar fisicamente a orientação do sensor para não mudar entre C0 e C1.

- [ ] **Step 2: Preparar C0**

Usar fixação nominal e velocidade estável. Ligar o motor por 30 s antes da primeira sessão.

- [ ] **Step 3: Capturar três sessões C0**

Exemplo:

```bash
python host/serial_capture.py --port /dev/ttyUSB0 --label C0 --session s01
python host/serial_capture.py --port /dev/ttyUSB0 --label C0 --session s02
python host/serial_capture.py --port /dev/ttyUSB0 --label C0 --session s03
```

- [ ] **Step 4: Preparar C1 de maneira segura e reprodutível**

Preferência para o primeiro cenário: alterar a rigidez da fixação da base de forma controlada, por exemplo utilizando dois pontos de fixação em C0 e um arranjo previamente definido menos rígido em C1, sem tocar no rotor e sem introduzir peças soltas.

- [ ] **Step 5: Capturar três sessões C1**

```bash
python host/serial_capture.py --port /dev/ttyUSB0 --label C1 --session s01
python host/serial_capture.py --port /dev/ttyUSB0 --label C1 --session s02
python host/serial_capture.py --port /dev/ttyUSB0 --label C1 --session s03
```

- [ ] **Step 6: Verificar tamanho das sessões**

Cada arquivo deve conter aproximadamente 6.000 amostras úteis, correspondente a 60 s em 100 Hz.

---

### Task 7: Aplicar critérios go/no-go da Fase 1

**Files:**
- Analyze: `data/raw/*.csv`

**Interfaces:**
- Consumes: seis CSVs C0/C1.
- Produces: decisão documentada `GO` ou `NO-GO` para iniciar janelamento/ML.

- [ ] **Step 1: Verificar frequência de cada sessão**

```bash
python host/analysis.py data/raw/C0_s01.csv
python host/analysis.py data/raw/C1_s01.csv
```

Critério inicial: frequência mediana entre `98 Hz` e `102 Hz`. Se ficar fora disso, corrigir aquisição antes de treinar ML.

- [ ] **Step 2: Verificar saturação**

Como o acelerômetro está configurado em ±4 g, investigar qualquer ocorrência persistente próxima a `-4 g` ou `+4 g` em `ax_g`, `ay_g` ou `az_g`. Saturação repetida exige aumentar a faixa ou mudar a montagem.

- [ ] **Step 3: Verificar repetibilidade de C0**

Comparar as distribuições de `magnitude_rms_g`, `magnitude_std_g` e `magnitude_ptp_g` entre C0_s01, C0_s02 e C0_s03. Elas não precisam ser idênticas, mas não devem parecer três regimes completamente diferentes.

- [ ] **Step 4: Verificar separabilidade C0/C1**

Há `GO` se pelo menos uma das métricas simples (`RMS`, desvio padrão ou pico-a-pico) apresentar mudança consistente entre C0 e C1 nas três repetições, sem depender de uma única sessão atípica.

- [ ] **Step 5: Fechar a Fase 1**

Somente com `GO` avançar para a Fase 2: janelamento definitivo, divisão treino/validação/teste por sessão e primeiro modelo supervisionado/anomaly detection.
