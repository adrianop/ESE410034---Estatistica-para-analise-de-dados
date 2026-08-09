# Fusão sensorial com Filtro de Kalman

Projeto da Avaliação C1 da disciplina ESE410034 — Estatística para Análise
de Dados. O sistema estima a inclinação de uma IMU MPU6050 por meio da fusão
das leituras do acelerômetro e do giroscópio com um Filtro de Kalman (KF).

O acelerômetro fornece uma referência absoluta de ângulo, mas é sensível a
ruídos e movimentos. O giroscópio responde bem no curto prazo, porém acumula
deriva quando sua velocidade angular é integrada. O filtro combina as vantagens
dos dois sensores e também estima o *bias* do giroscópio.

## Identificação

| Integrante | Matrícula |
| --- | --- |
| Adriano Pertile | 202601914 |
| Aluizio Augusto Kleine Kirschner | 202603883 |

## Estrutura do projeto

- `src/kalman.py`: implementação genérica do Filtro de Kalman (Parte A);
- `src/simulacao.py`: simulação da fusão sensorial (Parte B);
- `src/main_pratica.py`: aquisição serial e visualização em tempo real (Parte C);
- `firmware/leitura_mpu6050.ino`: firmware do Arduino Mega 2560;
- `Simulação/`: registros da simulação e da montagem;
- `docs/`: relatório e figuras do trabalho.

## Instalação

Requisitos: Git e Python 3.10 ou mais recente. Para a Parte C, também são
necessários Arduino IDE, Arduino Mega 2560 e um módulo MPU6050 (GY-521).

1. Clone do repositório:

   ```bash
   git clone https://github.com/adrianop/ESE410034---Estatistica-para-analise-de-dados.git
   cd ESE410034---Estatistica-para-analise-de-dados/"Avaliação C1"
   ```

2. Crie e ative um ambiente virtual.

   No Windows (PowerShell):

   ```powershell
   py -m venv .venv
   .\.venv\Scripts\Activate.ps1
   ```

   No Linux ou macOS:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

3. Instale as dependências usando o arquivo localizado na raiz do repositório:

   ```bash
   python -m pip install --upgrade pip
   python -m pip install -r requirements.txt
   ```


## Parte A — Filtro de Kalman genérico

A classe `KalmanFilter`, em `src/kalman.py`, recebe as matrizes do modelo no
construtor e disponibiliza separadamente as operações de predição (`predict`)
e atualização (`update`). Ela é utilizada tanto pela simulação quanto pela
aplicação com hardware.

## Parte B — Execução da simulação

Com o ambiente virtual ativado e a partir do diretório `Avaliação C1`, execute:

```bash
python src/simulacao.py
```

A simulação gera dados sintéticos do acelerômetro e do giroscópio, compara-os
com o ângulo real e exibe os resultados do filtro. As figuras são gravadas em
`docs/figures/`. Feche as janelas dos gráficos para o programa terminar e exibir
as métricas no terminal.

Para testar a interface da Parte C sem conectar o hardware, execute:

```bash
python src/main_pratica.py --demo
```

## Parte C — Montagem e execução com hardware

### Ligações

Faça as conexões com o circuito desligado:

| MPU6050 (GY-521) | Arduino Mega 2560 |
| --- | --- |
| VCC | 5 V |
| GND | GND |
| SDA | Pino 20 (SDA) |
| SCL | Pino 21 (SCL) |
| INT | Não conectado |

Confira a tensão aceita pelo seu módulo antes de alimentá-lo. O esquema usado
no projeto está em [`Simulação/circuito.png`](Simula%C3%A7%C3%A3o/circuito.png).

### Carregamento do firmware

1. Abra `firmware/leitura_mpu6050.ino` na Arduino IDE.
2. Em **Ferramentas > Placa**, selecione **Arduino Mega or Mega 2560**.
3. Selecione o processador **ATmega2560** e a porta serial correspondente.
4. Conecte o Arduino e clique em **Carregar**. A biblioteca `Wire`, usada pelo
   firmware, já faz parte do núcleo do Arduino e não exige instalação adicional.
5. Aguarde a calibração mantendo a IMU imóvel e feche o Monitor Serial antes de
   iniciar o programa Python, pois somente um programa pode usar a porta.

O firmware transmite a cada 20 ms (50 Hz) uma linha no formato:

```text
gyro_rate,acc_angle
```

Os valores correspondem à velocidade angular em graus/s e ao ângulo de *roll*
em graus. O programa também aceita o protocolo estendido
`t_ms,ax,ay,az,gx,gy,gz`.

### Execução

Descubra a porta no Gerenciador de Dispositivos ou na Arduino IDE e execute,
substituindo `COM5` pela porta correta:

```powershell
python src/main_pratica.py --port COM5
```

Ao iniciar, mantenha a IMU imóvel durante as 150 amostras de calibração
(aproximadamente 3 segundos). Essa posição será considerada 0°. A interface
mostra simultaneamente os últimos 10 segundos, 60 segundos e uma hora.

Opções úteis incluem `--calibration-samples 250`, `--auto-noise` para estimar
a variância do acelerômetro, `--refresh-ms 100` para ajustar a atualização da
tela e `--no-calibration` apenas para diagnóstico.

Se aparecer `ERRO_MPU6050_NAO_ENCONTRADO`, confira a alimentação, o GND comum,
os pinos I2C e o endereço do módulo. Se a porta estiver ocupada, feche o Monitor
Serial e qualquer outro programa que esteja conectado ao Arduino.
