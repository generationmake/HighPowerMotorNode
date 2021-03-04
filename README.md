# HighPowerMotorNode
UAVCAN node with HighPowerMotorFeatherWing, CANFeatherWing and Adafruit Feather M0

## used libraries

  * https://github.com/107-systems/107-Arduino-UAVCAN
  * https://github.com/107-systems/107-Arduino-MCP2515

## UAVCAN

### Subject IDs

```
Heartbeat - pub
2001 - pub - Bit    - Emergency Switch Status
2002 - pub - Real32 - InputVoltage (0.0V..52.8V)
2101 - sub - Real32 - Speed A (-1.0..+1.0)
2102 - sub - Real32 - Speed B (-1.0..+1.0)
2011 - pub - Real32 - Speed feedback A (-1.0..+1.0)
2012 - pub - Real32 - Speed feedback B (-1.0..+1.0)
2021 - pub - Real32 - Current A (-10.0A..+10.0A)
2022 - pub - Real32 - Current A (-10.0A..+10.0A)
```

