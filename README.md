ESP-IDF BQ27441 Driver
====================
Direct port of the arduino library to ESP-IDF v4.0

The bus must be configured manually in your program and started before you call begin();

Note that i2c_set_timeout_() is needed to prevent errors due to clock stretching.

See example in main for usage details.

*Not thread safe.*

# References

1. bq27441-g1.pdf (https://www.ti.com/lit/ds/symlink/bq27441-g1.pdf)
2. sluuac9a.pdf (https://www.ti.com/lit/ug/sluuac9a/sluuac9a.pdf?ts=1712369739227)
