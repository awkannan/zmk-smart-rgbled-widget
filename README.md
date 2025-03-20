# LED indicators using "Smart" LED

Note: this was adapted from [caksoylar's zmk-rgbled-widget](https://github.com/caksoylar/zmk-rgbled-widget) to function with Smart LEDs typically used for underglow.

This is a [ZMK module](https://zmk.dev/docs/features/modules) containing a simple widget that utilizes a string of Smart RGB LEDs.
It is used to indicate battery level and BLE connection status in a minimalist way.

## Features

<details>
  <summary>Short video demo</summary>
  See below video for a short demo, running through power on, profile switching and power offs.

  https://github.com/caksoylar/zmk-rgbled-widget/assets/7876996/cfd89dd1-ff24-4a33-8563-2fdad2a828d4
</details>

### Battery status 

- Blink 🟢/🟡/🔴 on boot depending on battery level, with thresholds [set](#configuration) by `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH` and `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW`
- Blink 🔴 on every battery level change if below critical battery level (`CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_CRITICAL`)

### Connection status

- Blink 🔵 for connected, 🟡 for open (advertising), 🔴 for disconnected profiles on boot after the battery blink, and following every BT profile switch (only on central side for splits)
- Blink 🔵 for connected, 🔴 for disconnected on peripheral side of splits

### Layer state

You can pick one of the following methods (off by default) to indicate the highest active layer:

- Enable `CONFIG_RGBLED_WIDGET_SHOW_LAYER_CHANGE` to show the highest active layer on every layer activation
  using a sequence of N cyan color blinks, where N is the zero-based index of the layer, or
- Enable `CONFIG_RGBLED_WIDGET_SHOW_LAYER_COLORS` to assign each layer its own color, which will remain on while that layer is the highest active layer

These layer indicators will only be active on the central part of a split keyboard, since peripheral parts aren't aware of the layer information.

NOTE: There is currently an unresolved issue with `CONFIG_RGBLED_WIDGET_SHOW_LAYER_COLORS` - the current widget doesn't have logic to prevent resetting RGB state to before layers were changed, so as a result, any RGB settings on layers will be overwritten.

If you need layer indicators, it's recommended to use `CONFIG_RGBLED_WIDGET_SHOW_LAYER_CHANGE` for now until this issue is fixed.

> [!TIP]
> Also see [below](#showing-status-on-demand) for keymap behaviors you can use to show the battery and connection status on demand.

## Installation

Add this module to your `config/west.yml` by adding a new entry to `remotes` and `projects`, and point your zmk source to the branch with the necessary core changes:

```yaml west.yml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: awkannan  # <-- new entry
      url-base: https://github.com/awkannan
  projects:
    - name: zmk # <-- modified entry
      remote: awkannan
      revision: develop
      import: app/west.yml
    - name: zmk-smart-rgbled-widget  # <-- new entry
      remote: awkannan
      revision: main
  self:
    path: config
```

For more information, including instructions for building locally, check out the ZMK docs on [building with modules](https://zmk.dev/docs/features/modules#building-with-modules).

## Showing status on demand

This module also defines keymap [behaviors](https://zmk.dev/docs/keymaps/behaviors) to let you show battery or connection status on demand:

```dts
#include <behaviors/rgbled_widget.dtsi>  // needed to use the behaviors

/ {
    keymap {
        ...
        some_layer {
            bindings = <
                ...
                &ind_bat  // indicate battery level
                &ind_con  // indicate connectivity status
                ...
            >;
        };
    };
};
```

When you invoke the behavior by pressing the corresponding key (or combo), it will trigger the LED color display.
This will happen on all keyboard parts for split keyboards, so make sure to flash firmware to all parts after enabling.

> [!NOTE]
> The behaviors can be used even when you use split keyboards with different controllers that don't all support the widget.
> Make sure that you use the `rgbled_adapter` shield (or enable `CONFIG_RGBLED_WIDGET` if not using the adapter) only for the keyboard parts that support it.

## Configuration

<details>
<summary>Expand to see available configuration options</summary>

| Name                                           | Description                                                                  | Default       |
| ---------------------------------------------- | ---------------------------------------------------------------------------- | ------------- |
| `CONFIG_RGBLED_WIDGET_INTERVAL_MS`             | Minimum wait duration between two blinks in ms                               | 500           |
| `CONFIG_RGBLED_WIDGET_BATTERY_BLINK_MS`        | Duration of battery level blink in ms                                        | 2000          |
| `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH`      | High battery level percentage                                                | 80            |
| `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_LOW`       | Low battery level percentage                                                 | 20            |
| `CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_CRITICAL`  | Critical battery level percentage, blink periodically if under               | 5             |
| `CONFIG_RGBLED_WIDGET_BATTERY_COLOR_HIGH`      | Color for high battery level (above `LEVEL_HIGH`)                            | Green (`2`)   |
| `CONFIG_RGBLED_WIDGET_BATTERY_COLOR_MEDIUM`    | Color for medium battery level (between `LEVEL_LOW` and `LEVEL_HIGH`)        | Yellow (`3`)  |
| `CONFIG_RGBLED_WIDGET_BATTERY_COLOR_LOW`       | Color for low battery level (below `LEVEL_LOW`)                              | Red (`1`)     |
| `CONFIG_RGBLED_WIDGET_BATTERY_COLOR_CRITICAL`  | Color for critical battery level (below `LEVEL_CRITICAL`)                    | Red (`1`)     |
| `CONFIG_RGBLED_WIDGET_CONN_BLINK_MS`           | Duration of BLE connection status blink in ms                                | 1000          |
| `CONFIG_RGBLED_WIDGET_CONN_COLOR_CONNECTED`    | Color for connected BLE connection status                                    | Blue (`4`)    |
| `CONFIG_RGBLED_WIDGET_CONN_COLOR_ADVERTISING`  | Color for advertising BLE connection status                                  | Yellow (`3`)  |
| `CONFIG_RGBLED_WIDGET_CONN_COLOR_DISCONNECTED` | Color for disconnected BLE connection status                                 | Red (`1`)     |
| `CONFIG_RGBLED_WIDGET_SHOW_LAYER_CHANGE`       | Indicate highest active layer on each layer change with a sequence of blinks | `n`           |
| `CONFIG_RGBLED_WIDGET_LAYER_BLINK_MS`          | Blink and wait duration for layer indicator                                  | 100           |
| `CONFIG_RGBLED_WIDGET_LAYER_COLOR`             | Color to use for layer indicator                                             | Cyan (`6`)    |
| `CONFIG_RGBLED_WIDGET_LAYER_DEBOUNCE_MS`       | Wait duration after a layer change before showing the highest active layer   | 100           |
| `CONFIG_RGBLED_WIDGET_SHOW_LAYER_COLORS`       | Indicate highest active layer with a constant configurable color per layer   | `n`           |
| `CONFIG_RGBLED_WIDGET_LAYER_0_COLOR`           | Color to use for the base layer                                              | Black (`0`)   |
| `CONFIG_RGBLED_WIDGET_LAYER_1_COLOR`           | Color to use for layer 1                                                     | Red (`1`)     |
| `CONFIG_RGBLED_WIDGET_LAYER_2_COLOR`           | Color to use for layer 2                                                     | Green (`2`)   |
| `CONFIG_RGBLED_WIDGET_LAYER_3_COLOR`           | Color to use for layer 3                                                     | Yellow (`3`)  |
| `CONFIG_RGBLED_WIDGET_LAYER_4_COLOR`           | Color to use for layer 4                                                     | Blue (`4`)    |
| `CONFIG_RGBLED_WIDGET_LAYER_5_COLOR`           | Color to use for layer 5                                                     | Magenta (`5`) |
| `CONFIG_RGBLED_WIDGET_LAYER_6_COLOR`           | Color to use for layer 6                                                     | Cyan (`6`)    |
| `CONFIG_RGBLED_WIDGET_LAYER_7_COLOR`           | Color to use for layer 7                                                     | White (`7`)   |
| `CONFIG_RGBLED_WIDGET_LAYER_xx_COLOR`          | Color to use for layer xx (change xx to the layer number to change)          | Black (`0`)   |

Color settings use the following integer values:

| Color        | Value |
| ------------ | ----- |
| Black (none) | `0`   |
| Red          | `1`   |
| Green        | `2`   |
| Yellow       | `3`   |
| Blue         | `4`   |
| Magenta      | `5`   |
| Cyan         | `6`   |
| White        | `7`   |

</details>

You can add these settings to your keyboard conf file to modify the config values, e.g. in `config/hummingbird.conf`:

```ini
CONFIG_RGBLED_WIDGET_INTERVAL_MS=250
CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_HIGH=50
CONFIG_RGBLED_WIDGET_BATTERY_LEVEL_CRITICAL=10
```

## Adding support in custom boards/shields

To be able to use this widget, you need to define and enable RGB underglow as shown in the [ZMK Docs](https://zmk.dev/docs/features/lighting#rgb-underglow)

Finally, turn on the widget in the configuration:

```ini
CONFIG_SMART_RGBLED_WIDGET=y
```
