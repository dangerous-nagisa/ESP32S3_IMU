# ICM42688 6-Axis MotionTracking (Accelerometer and Gyroscope)

[![Component Registry](https://components.espressif.com/components/espressif/ICM42688/badge.svg)](https://components.espressif.com/components/espressif/ICM42688)
![maintenance-status](https://img.shields.io/badge/maintenance-passively--maintained-yellowgreen.svg)

C driver for Invensense ICM42688 6-axis gyroscope and accelerometer based on I2C communication.

## Features

- Get 3-axis accelerometer and 3-axis gyroscope data, either raw or as floating point values. 
- Read temperature from ICM42688 internal temperature sensor.
- Configure gyroscope and accelerometer sensitivity.
- ICM42688 power down mode.

## Limitations

- Only I2C communication is supported.

## Get Started

This driver, along with many other components from this repository, can be used as a package from [Espressif's IDF Component Registry](https://components.espressif.com). To include this driver in your project, run the following idf.py from the project's root directory:

```
    idf.py add-dependency "espressif/ICM42688==*"
```

Another option is to manually create a `idf_component.yml` file. You can find more about using .yml files for components from [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## See Also
* [ICM42688 datasheet](https://item.szlcsc.com/datasheet/ICM-42688P-HXY/48681527.html?spm=sc.gbn.xds.a&lcsc_vid=RllXVgUEFVNcU1UERlZdAVUHRFBeAgVURlZaXgdQTlgxVlNQT1lXV1JXRVJYVzsOAxUeFF5JWBIBSRccGwIdBEoFGAxBAAgJFQACSQwSGg0%3D)
