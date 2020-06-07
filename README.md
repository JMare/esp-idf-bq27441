ESP-IDF BQ27441 Driver
====================
Direct port of the arduino library to ESP-IDF v4.0

The bus must be configured manually in your program and started before you call begin();

Note that i2c_set_timeout_() is needed to prevent errors due to clock stretching.

See example in main for usage details.

*Not thread safe.*
