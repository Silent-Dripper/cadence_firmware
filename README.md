# Cadence Firmware

The core firmware is in `./cadence-firmware`. This is the code that should be uploaded in a production environment.

There are a few test sketches in `./test` that verify the functionality in the core `cadence-firmware`. For each of these sketches, there may or may not be docs at the top of the respective `.ino`s. 


## Contributing

PR's are welcome to increase code quality, improve heartbeat detection or support more hardware platforms. A few notes:

* Use doxygen style docstrings. I got stubs for these using [this](https://github.com/cschlosser/doxdocgen) tool.
* `clang-format` using default settings is used to enforce code style.