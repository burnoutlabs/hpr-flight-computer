# Welcome to the HPR-Flight-Computer Docs

## External contracts

What the flight computer reads and writes outside its own memory, which other boards, tools and
files depend on:

- [EEPROM map](eeprom-map.md): what each EEPROM address holds, and what reads and writes it.
- [Settings files](settings-files.md): `EEPROMsettings.txt` and `Settings.txt`, read by
  position, and the device ID codes.
- [SD card flight log](sd-flight-log.md): the CSV columns, the `fltEvents` bit string and the
  footer.
- [Telemetry packets](telemetry-packets.md): the radio packet layout and the flight event codes.

## Operations Flow Chart

Source can be found on Cameron Brooks' Google Drive: [drive.google.com/file/d/1UoUj555KsNLJwYxSnVGjF17hvRSPmnp4/view?usp=drive_link](https://drive.google.com/file/d/1UoUj555KsNLJwYxSnVGjF17hvRSPmnp4/view?usp=drive_link)

![Operations_Flow_Chart_v1](Operations_Flow_Chart_v1.png)
