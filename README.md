# INDIDuino - Stepper

This is our version of the Stepper module with some minor alterations, the main one being able to set multipliers for
all the parameters given that the maximum limit of 16000 is not enough for our use case.

# Usage

Flash the firmware and then run

```bash
INDISKEL=$(pwd)/stepper_sk.xml indiserver -v indi_duino
```
