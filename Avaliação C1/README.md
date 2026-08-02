# MPU6050 com filtro de Kalman

O firmware envia a cada 20 ms:

```text
t_ms,ax,ay,az,gx,gy,gz
```

`ax`, `ay` e `az` estão em g; `gx`, `gy` e `gz`, em graus/s. O programa
Python converte esses valores em roll (padrão) ou pitch e funde acelerômetro e
giroscópio com o filtro de Kalman.

## Ligação e execução

No Arduino Mega 2560, use SDA no pino 20 e SCL no pino 21. Carregue `firmware/leitura_mpu6050.ino` e feche o Monitor Serial antes de
abrir a porta pelo Python.

Instale as dependências, se necessário:

```powershell
py -m pip install numpy matplotlib pyserial
```

Descubra a porta no Gerenciador de Dispositivos ou na Arduino IDE e execute:

```powershell
py src/main_pratica.py --port COM5
```

Para pitch:

```powershell
py src/main_pratica.py --port COM3 --axis pitch
```

Ao iniciar, mantenha a IMU imóvel durante as 150 amostras de calibração
(aproximadamente 3 segundos). A posição nesse momento será considerada 0°.
Para conferir a interface sem hardware:

```powershell
py src/main_pratica.py --demo
```

Opções úteis: `--calibration-samples 250`, `--auto-noise` para estimar a
variância do acelerômetro, `--refresh-ms 100` para definir a atualização da
tela e `--no-calibration` apenas para diagnóstico. A aquisição ocorre em uma
thread própria a cada 20 ms (50 Hz), portanto não é interrompida enquanto o
Matplotlib desenha o gráfico. Se um quadro demorar, todas as amostras que
chegaram nesse período são processadas no quadro seguinte.

A interface mostra simultaneamente três históricos: últimos 10 segundos,
últimos 60 segundos e última hora. O histórico é mantido em resolução completa
por uma hora; o painel longo limita a quantidade de pontos desenhados para não
reduzir a velocidade da aquisição em tempo real.

Se aparecer `MPU6050 nao respondeu`, confira alimentação, GND comum, os pinos
I2C e o endereço do módulo. Se a porta estiver ocupada, feche o Monitor Serial.
