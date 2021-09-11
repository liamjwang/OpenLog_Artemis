##Why switch to PlatformIO?
- You won't need to install the 20+ Arduino dependencies manually!
- Specify build flags like `ICM_20948_USE_DMP` in the platformio.ini instead of modifying library headers!
- Use fixed version numbers for dependencies so your project won't break when a library updates!


## Setup for Development with PlatformIO

- Install PlatformIO Core `https://platformio.org/install`
- Follow these directions to install the apollo3blue platform: [https://github.com/nigelb/platform-apollo3blue#install](https://github.com/nigelb/platform-apollo3blue#install)

## Build and Upload

Open the project in any [PlatformIO compatible IDE](https://platformio.org/install/integration) and upload the firmware to your OLA.

Or, you could upload to the OLA using the command line:
- `cd <repo location on your computer>/OpenLog_Artemis`
- `pio run --target upload`

See [https://docs.platformio.org/en/latest/what-is-platformio.html](https://docs.platformio.org/en/latest/what-is-platformio.html) for more information.