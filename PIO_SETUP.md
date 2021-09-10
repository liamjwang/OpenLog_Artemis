## Setup for Development with PlatformIO

_No need to install Arduino dependencies manually!_

- Install PlatformIO Core `https://platformio.org/install`
- Install the Sparkfun Arduino Apollo3 framework in your .platformio directory (use PowerShell if you're using Windows)
  - `cd ~/.platformio/packages`
  - `git clone --recurse-submodules --depth 1 --branch v1.2.3 https://github.com/sparkfun/Arduino_Apollo3.git framework-arduinoapollo3@1.2.3`
- Install the apollo3blue platform ([https://github.com/nigelb/platform-apollo3blue](https://github.com/nigelb/platform-apollo3blue))
  - `cd ~/.platformio/platforms`
  - `git clone https://github.com/nigelb/platform-apollo3blue.git apollo3blue`

## Build and Upload

Open the project in any [PlatformIO compatible IDE](https://platformio.org/install/integration) and upload the firmware to your OLA.

Or, you could upload to the OLA using the command line:
- `cd <repo location on your computer>/OpenLog_Artemis`
- `pio run --target upload`

See [https://docs.platformio.org/en/latest/what-is-platformio.html](https://docs.platformio.org/en/latest/what-is-platformio.html) for more information.