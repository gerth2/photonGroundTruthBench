"""MPU6050 accelerometer / gyroscope I²C driver for RobotPy 2027.

Register map follows the MPU-6000/MPU-6050 register map datasheet (RM-MPU-60XA).
"""

import math
import wpilib


MPU6050_DEFAULT_ADDRESS = 0x68

REG_PWR_MGMT_1 = 0x6B
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT_H = 0x3B
REG_GYRO_XOUT_H = 0x43

# Scale factors for each full-scale range (datasheet §4.4 & §4.5).
GYRO_SCALE = {
    250: 131.0,
    500: 65.5,
    1000: 32.8,
    2000: 16.4,
}

ACCEL_SCALE = {
    2: 16384.0,
    4: 8192.0,
    8: 4096.0,
    16: 2048.0,
}


class MPU6050:
    def __init__(
        self,
        port: int | wpilib.I2C.Port = wpilib.I2C.Port.PORT_0,
        address: int = MPU6050_DEFAULT_ADDRESS,
        gyro_scale_dps: int = 250,
        accel_scale_g: int = 2,
    ) -> None:
        port_value: wpilib.I2C.Port = (
            port if isinstance(port, wpilib.I2C.Port) else wpilib.I2C.Port(port)
        )
        self._i2c = wpilib.I2C(port_value, address)
        self._gyro_scale = GYRO_SCALE[gyro_scale_dps]
        self._accel_scale = ACCEL_SCALE[accel_scale_g]
        self._deg_to_rad = math.pi / 180.0

        self._i2c.write(REG_PWR_MGMT_1, 0x00)  # wake from sleep (§4.30)

        gyro_config = {250: 0, 500: 8, 1000: 16, 2000: 24}[gyro_scale_dps]
        self._i2c.write(REG_GYRO_CONFIG, gyro_config)

        accel_config = {2: 0, 4: 8, 8: 16, 16: 24}[accel_scale_g]
        self._i2c.write(REG_ACCEL_CONFIG, accel_config)

    def _read_i16(self, register: int) -> int:
        data = bytearray(2)
        self._i2c.read(register, data)
        return int.from_bytes(data, byteorder="big", signed=True)

    def read_accel_mps2(self) -> tuple[float, float, float]:
        x = self._read_i16(REG_ACCEL_XOUT_H) / self._accel_scale * 9.80665
        y = self._read_i16(REG_ACCEL_XOUT_H + 2) / self._accel_scale * 9.80665
        z = self._read_i16(REG_ACCEL_XOUT_H + 4) / self._accel_scale * 9.80665
        return (x, y, z)

    def read_gyro_radps(self) -> tuple[float, float, float]:
        x = self._read_i16(REG_GYRO_XOUT_H) / self._gyro_scale * self._deg_to_rad
        y = self._read_i16(REG_GYRO_XOUT_H + 2) / self._gyro_scale * self._deg_to_rad
        z = self._read_i16(REG_GYRO_XOUT_H + 4) / self._gyro_scale * self._deg_to_rad
        return (x, y, z)
