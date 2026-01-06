# Self-balancing-Balboa
# Balboa Self-Balancing Robot Project

Dette prosjektet gikk ut på å få en **Balboa 32U4-robot** til å **selvbalansere i over 10 sekunder** ved hjelp av sensorer og et PID-kontrollsystem.

## Systembeskrivelse
- **Robotplattform:** Balboa 32U4  
- **Sensorer:** LSM6 IMU (akselerometer og gyroskop)  
- **Aktuatorer:** Motorsystem med to hjul  
- **Kontrollplattform:** Innebygd 32U4 mikrokontroller med motor-encoder feedback  
- **Ytelse:** Loop-frekvens på ca. 200 Hz

## Kontrollmetode
- **Kontrollsystem:** PID-kontroller  
- **Parametere:**  
  - Proportional (Kp) = 25.0  
  - Integral (Ki) = 120  
  - Derivative (Kd) = 0.7  

- **Filter:** Komplementært filter som kombinerer akselerometer og gyroskop for å estimere robotens tilt-vinkel.  
- **PID logikk:**  
  - Feil = tilt-vinkel i grader fra oppreist posisjon  
  - Integral og derivat beregnes kontinuerlig  
  - Motorhastighet justeres basert på PID-utgang

## Validering
- Gyroskopet ble kalibrert for bias før start.  
- Komplementært filter og PID-kontroller ble testet i sanntid på roboten.  
- Robotens evne til å selvbalansere ble observert og bekreftet med videoopptak.

## Resultat
Robotens selvbalansering var stabil i over 10 sekunder, noe som demonstrerer effektiviteten til PID-kontrolleren og filterkombinasjonen brukt.

