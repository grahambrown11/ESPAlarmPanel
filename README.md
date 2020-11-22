# ESP Alarm Panel Module

The code using the [dscKeybusInterface](https://github.com/taligentx/dscKeybusInterface) library that runs on a D1 Mini for my DSC Alarm Panel. It's combo of the VirtualKeypad-Web & HomeAssistant-MQTT examples, altered for my needs.

## Build & Upload to D1 Mini
This project uses PlatformIO, I use the PlatformIO IDE extension in VSCode - the build and upload buttons are super convenient, but I've not found how to upload the website data so run `platformio run --target uploadfs` in the PlatformIO Terminal to upload the website to the D1 Mini.

Before you can build you'll need copy the `secrets.h.example` to `secrets.h` and set the values for your network and panel.

## HomeAssistant example alarm config:

``` YAML
alarm_control_panel:
  - platform: mqtt
    name: "Security System"
    state_topic: "dsc/Get/Partition1"
    availability_topic: "dsc/Status"
    command_topic: "dsc/Set"
    code: "1234"
    code_arm_required: true
    payload_disarm: "1D"
    payload_arm_home: "1S"
    payload_arm_away: "1A"

binary_sensor:
  - platform: mqtt
    name: "Security Trouble"
    state_topic: "dsc/Get/Trouble"
    device_class: "problem"
    payload_on: "1"
    payload_off: "0"
  - platform: mqtt
    name: "Back door"
    state_topic: "dsc/Get/Zone1"
    device_class: "door"
    payload_on: "1"
    payload_off: "0"
  - platform: mqtt
    name: "Dining Room"
    state_topic: "dsc/Get/Zone2"
    device_class: "motion"
    payload_on: "1"
    payload_off: "0"
  - platform: mqtt
    name: "Lounge"
    state_topic: "dsc/Get/Zone3"
    device_class: "motion"
    payload_on: "1"
    payload_off: "0"
  - platform: mqtt
    name: "Front door"
    state_topic: "dsc/Get/Zone4"
    device_class: "door"
    payload_on: "1"
    payload_off: "0"
  - platform: mqtt
    name: "Main bedroom"
    state_topic: "dsc/Get/Zone5"
    device_class: "motion"
    payload_on: "1"
    payload_off: "0"
  - platform: mqtt
    name: "Passage"
    state_topic: "dsc/Get/Zone6"
    device_class: "motion"
    payload_on: "1"
    payload_off: "0"
  - platform: mqtt
    name: "Garage"
    state_topic: "dsc/Get/Zone7"
    device_class: "motion"
    payload_on: "1"
    payload_off: "0"
```

More details found at [MQTT Alarm Control Panel](https://www.home-assistant.io/components/alarm_control_panel.mqtt/) on HomeAssistants website.
