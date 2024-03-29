# EZ-PD&trade; PMG1 MCU: USB PD sink CAPSENSE&trade;

This code example demonstrates USB Type-C attach detection and USB Power Delivery contract negotiation using an EZ-PD&trade; PMG1-S3 MCU device. This code example can negotiate up to a 140 W (28 V at 5 A) Extended Power Range (EPR) Power Delivery contract with a 140 W EPR capable USB-C source. The code example also features a 5-segment CAPSENSE&trade; slider and two CAPSENSE&trade; buttons. Each CAPSENSE&trade; sensor toggles an LED and also triggers renegotiation of the Power Delivery contract to get different VBUS voltage based on the touch input from the user.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-pmg1-usbpd-sink-capsense)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyMzM2NDUiLCJTcGVjIE51bWJlciI6IjAwMi0zMzY0NSIsIkRvYyBUaXRsZSI6IkVaLVBEJnRyYWRlOyBQTUcxIE1DVTogVVNCIFBEIHNpbmsgQ0FQU0VOU0UmdHJhZGU7IiwicmlkIjoicHNociIsIkRvYyB2ZXJzaW9uIjoiNC4wLjAiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiV0lSRUQiLCJEb2MgRmFtaWx5IjoiVFlQRS1DIn0=)

## Requirements

- [ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0)
- Board support package (BSP) minimum required version: 3.0.0
- Programming language: C
- Associated parts: All [EZ-PD&trade; PMG1-S3 MCU](https://www.infineon.com/PMG1) parts

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm® embedded compiler v10.3.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; compiler v6.16 (`ARM`)
- IAR C/C++ compiler v9.30.1 (`IAR`)

## Supported kits (make variable 'TARGET')

- [EZ-PD&trade; PMG1-S3 MCU Prototyping Kit](https://www.infineon.com/CY7113) (`PMG1-CY7113`) – Default value of `TARGET`

## Hardware setup

1. Connect the board to your PC using the USB cable through the KitProg3 USB connector. Use the cable for programming the EZ-PD&trade; PMG1 device and during debugging.

2. Connect the USB PD port to the USB-C power adapter or your PC using the USB Type-C cable. Use the cable for the USB Power Delivery source and it powers the user LED.

<b>Note:</b> To test the EPR feature, a 140 W USB-C power adapter, and a 140 W USB-C to USB-C cable are needed.

See the kit user guide for details on configuring the board.


## Software setup

This example requires no additional software or tools.


## Using the code example

Create the project and open it using one of the following:

<details><summary><b>In Eclipse IDE for ModusToolbox&trade; software</b></summary>

1. Click the **New Application** link in the **Quick Panel** (or, use **File** > **New** > **ModusToolbox&trade; Application**). This launches the [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool.

2. Pick a kit supported by the code example from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library Manager, click the link from the **Quick Panel**.

   You can also just start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialog, choose the example by enabling the checkbox.

4. (Optional) Change the suggested **New Application Name**.

5. The **Application(s) Root Path** defaults to the Eclipse workspace which is usually the desired location for the application. If you want to store the application in a different location, you can change the *Application(s) Root Path* value. Applications that share libraries should be in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>

<details><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox&trade; software provides the Project Creator as both a GUI tool and the command line tool, "project-creator-cli". The CLI tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; software install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the "project-creator-cli" tool. On Windows, use the command line "modus-shell" program provided in the ModusToolbox&trade; software installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; software tools. You can access it by typing `modus-shell` in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The "project-creator-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the `<id>` field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the `<id>` field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br />

The following example clones the "[mtb-example-pmg1-usbpd-sink-capsense](https://github.com/Infineon/mtb-example-pmg1-usbpd-sink-capsense)" application with the desired name "UsbPdSinkCapSense" configured for the *PMG1-CY7113* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id PMG1-CY7113 --app-id mtb-example-pmg1-usbpd-sink-capsense --user-app-name UsbPdSinkCapSense --target-dir "C:/mtb_projects"
   ```

**Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can invoke the Library Manager GUI tool from the terminal using `make library-manager` command or use the Library Manager CLI tool "library-manager-cli" to change the BSP.

The "library-manager-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--add-bsp-name` | Name of the BSP that should be added to the application | Required
`--set-active-bsp` | Name of the BSP that should be as active BSP for the application | Required
`--add-bsp-version`| Specify the version of the BSP that should be added to the application if you do not wish to use the latest from manifest | Optional
`--add-bsp-location`| Specify the location of the BSP (local/shared) if you prefer to add the BSP in a shared path | Optional

<br>

Following example adds the PMG1-CY7113 BSP to the already created application and makes it the active BSP for the app:

   ```
    ~/ModusToolbox/tools_3.0/library-manager/library-manager-cli --project "C:/mtb_projects/UsbPdSinkCapSense" --add-bsp-name PMG1-CY7113 --add-bsp-version "latest-v3.X" --add-bsp-location "local"

    ~/ModusToolbox/tools_3.0/library-manager/library-manager-cli --project "C:/mtb_projects/UsbPdSinkCapSense" --set-active-bsp APP_PMG1-CY7113
   ```

</details>

<details><summary><b>In third-party IDEs</b></summary>

Use one of the following options:

- **Use the standalone [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool:**

   1. Launch Project Creator from the Windows Start menu or from *{ModusToolbox&trade; software install directory}/tools_{version}/project-creator/project-creator.exe*.

   2. In the initial **Choose Board Support Package** screen, select the BSP, and click **Next**.

   3. In the **Select Application** screen, select the appropriate IDE from the **Target IDE** drop-down menu.

   4. Click **Create** and follow the instructions printed in the bottom pane to import or open the exported project in the respective IDE.

<br>

- **Use command-line interface (CLI):**

   1. Follow the instructions from the **In command-line interface (CLI)** section to create the application.

   2. Export the application to a supported IDE using the `make <ide>` command.

   3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Ensure that the steps listed in the [Hardware setup](#hardware-setup) section are complete.

2. Ensure that the jumper shunt on the power selection jumper (J5) is placed at positions 2-3 while programming the kit.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade; software</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming the kit, change the positions on the power selection jumper (J5) to 1-2 to power the kit through the USB PD port. Do not change the jumper (J5) position while the cables are connected to the power source.

5. Observe that the user LED (LED3) on the board blinks at different rates depending on the type of power adapter connected:

   - If a USB Type-C power adapter or a Standard Downstream Port (SDP) is connected, the LED toggles every 5 seconds.

   - If a Dedicated Charging Port (DCP) is connected, the LED toggles every 7.5 seconds.

   - If a Charging Downstream Port (CDP) is connected, the LED toggles every 10 seconds.

   - If a power adapter supporting USB Power Delivery is connected and a 5V contract was negotiated, the LED toggles every 2 seconds.

   - If a 9V power delivery contract was negotiated, the LED toggles every 1.1 seconds.

   - If a 12V power delivery contract was negotiated, the LED toggles every 0.83 seconds.

   - If a 15V power delivery contract was negotiated, the LED toggles every 0.67 seconds.

   - If a 20V power delivery contract was negotiated, the LED toggles every 0.5 seconds.

   - If the power delivery contract negotitated is more than 20V, the LED toggles every 0.35 seconds.

6. Do the following to test the CAPSENSE&trade; functionality:

   - Touch **BTN1** to turn LED5 ON: Negotiates up to 20 V at 900 mA contract Standard Power Range (SPR) Power Delivery contract.

   - Touch **BTN2** to turn LED6 ON: Negotiates up to 28 V at 5 A (140 W) PR Power Delivery contract with a 140 W USB-C power adapter. 

   - Touch **SLD1** to turn LED7 ON: Negotiates up to 5 V at 900 mA SPR Power Delivery contract.

   - Touch **SLD2** to turn LED8 ON: Negotiates up to 9 V at 900 mA SPR Power Delivery contract.

   - Touch **SLD3** to turn LED9 ON: Negotiates up to 12 V at 900 mA SPR Power Delivery contract.

   - Touch **SLD4** to turn LED10 ON: Negotiates up to 15 V at 900 mA SPR Power Delivery contract.

   - Touch **SLD5** to turn LED11 ON: Negotiates up to 20 V at 900 mA SPR Power Delivery contract.

   <b>Note:</b> Ensure the USB-C power adapter supports high voltages when testing the renegotiation for high VBUS voltages. In absence of a 140 W USB-C power adapter, pressing BTN2 will only turn LED6 ON. There will not be any Power Delivery contract negotiation.

## Debugging

You can debug the example to step through the code. In the IDE, use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. Ensure that the board is connected to your PC using the USB cable through the KitProg3 USB connector and the jumper shunt on the power selection jumper (J5) is placed at positions 1-2. See the "Debug mode" section in the kit user guide for debugging the application on the CY7113 prototyping kit.

For more details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

## Design and implementation

EZ-PD&trade; PMG1 MCU devices support a USB PD block that integrates Type-C terminations, comparators, and the Power Delivery transceiver required to detect the attachment of a partner device and negotiate power contracts with it.

On reset, the USB PD block initializes the following settings:

   - The receiver clock input of the block is connected to a 12-MHz PERI-derived clock.

   - The transmitter clock input of the block is connected to a 600-kHz PERI-derived clock.

   - The SAR ADC clock input of the block is connected to a 1-MHz PERI-derived clock.

   - The SAR ADC in the USB PD block is configured to measure the VBUS_TYPE-C voltage through an internal divider.

This application uses the PDStack middleware library in an Upstream Facing Port (UFP) - sink configuration. EZ-PD&trade; PMG1 MCU devices have a dead-battery Rd termination, which ensures that a USB-C source/charger connected to it can detect the presence of a sink even when the EZ-PD&trade; PMG1 MCU device is not powered.

The CAPSENSE&trade; middleware library is used to initialize the CSD block, perform widget scan, and its processing.

**Figure 1. Firmware flowchart**

<img src = "images/fwflow.png" width = "300"/>
<br>

The PDStack middleware library configures the USB PD block on the EZ-PD&trade; PMG1 MCU device to detect Type-C connection state changes and USB PD messages, and notify the stack through callback functions. The callback function registers the pending tasks, which are then handled by PDStack through the `Cy_PdStack_Dpm_Task` function. This function is expected to call at appropriate times from the main processing loop of the application.

**Figure 2. PDStack task flowchart**

<img src = "images/pdstacktask.png" width = "600"/>

<br>

The PDStack middleware library implements the state machines defined in the [USB Type-C Cable and Connector](https://www.usb.org/document-library/usb-type-cr-cable-and-connector-specification-revision-20) and the [USB Power Delivery](https://www.usb.org/document-library/usb-power-delivery) specifications. PDStack consists of the following main modules:

- **Type-C Manager:** Detecting a Type-C connection and identifying the type of connection. It uses the configurable Rp/Rd terminations provided by the USB PD block and the internal line state comparators. The Type-C Manager implements the state machines defined in the USB Type-C Cable and Connector specification and provides the following functionality:

    - *Manage CC terminations*: Applies Rp/Rd terminations according to the port role

    - *Attach detection*: Performs the required debounce and determine the type of device attached

    - *Detach detection*: Monitors the CC line and VBus for detecting a device detach

- **Protocol Layer:** Forms the messages used to communicate between a pair of ports/cable plugs. It is responsible for forming capabilities messages, requests, responses, and acknowledgements. It receives inputs from the Policy Engine indicating which messages to send and relays the responses back to the policy engine.

- **Policy Engine:** Provides a mechanism to monitor and control the USB Power Delivery system within a particular consumer, provider, or cable plug. It implements the state machines defined in the *USB Power Delivery* specification and contains implementations of all PD Atomic Message Sequences (AMS). It interfaces with the protocol layer for PD message transmission/reception for controlling the reception of message types according to conditions such as the current state of the port. It also interfaces with the Type-C Manager for error conditions like Type-C error recovery.

- **Device Policy Manager (DPM):** Provides an interface to the application layer to initialize, monitor, and configure the PDStack middleware operation. The DPM provides the following functionality:

    - Initialize the Policy Engine and Type-C Manager

    - Start the Type-C state machine followed by the Policy Engine state machine

    - Stop and disable the Type-C port

    - Allow entry/exit from deep sleep to achieve low power based on the port status

    - Provide APIs for the application to send PD/Type-C commands

    - Provide event callbacks to the application for application-specific handling

The PDStack library uses a set of callbacks registered by the application to perform board-specific tasks such as turning the consumer power path ON/OFF and identifying the optimal source power profile to be used for charging. In this example, these functions are implemented using the appropriate APIs provided as a part of the Peripheral Driver Library (PDL).

The stack also provides notification of various connection and PD policy state changes so that the rest of the system can be configured as required. These events are used by the example application to implement a separate USB Battery Charging 1.2 sink state machine, which distinguishes between a standard downstream port (SDP), charging downstream port (CDP), and dedicated charging port (DCP).

The BC 1.2 sink state machine is activated only when the power source connected does not support USB Power Delivery. The GPIO connected to LED3 on the board is toggled at different rates to indicate the type of power source, which is detected by the PDStack library and the BC 1.2 sink state machine.

The application initiates an EPR mode entry request after an SPR contract establishes if the source is EPR capable. If the EPR mode entry is successful, the EPR sink maintains regular communication with the EPR source to allow EPR mode to continue.

The application tries to keep the EZ-PD&trade; PMG1 MCU device in deep sleep where all clocks are disabled and only limited hardware blocks are enabled, for most of its working time. Interrupts in the USB PD block are configured to detect any changes that happen while the device is in sleep and wake it up for further processing.

An overvoltage (OV) comparator in the USB PD block is used to detect cases where the power source is supplying incorrect voltage levels and automatically shut down the power switches to protect the rest of the components on the board.

### Compile-time configurations

Customize the EZ-PD&trade; PMG1 MCU USB PD sink application functionality through a set of compile-time parameters that can be turned ON/OFF through *config.h* or Makefile.

 Macro name          | Description                           | Allowed values 
 :------------------ | :------------------------------------ | :-------------
 `CY_PD_SINK_ONLY`     | Specifies that the application supports only the USB PD sink (consumer) role | Set to 1u
 `NO_OF_TYPEC_PORTS`   | Specifies the number of USB-C ports supported | Set to 1u
 `CY_PD_REV3_ENABLE`   | Enables USB PD Revision 3.1 support | 1u or 0u
 `CY_PD_EPR_ENABLE`    | Enables EPR Sink support | 1u or 0u
 `PD_PDO_SEL_ALGO`     | Specifies the algorithm to be used while selecting the best source capability to power the board | 0u – Pick the source PDO delivering the maximum power <br>1u – Pick the fixed source PDO delivering the maximum power <br>2u – Pick the fixed source PDO delivering the maximum current<br>3u – Pick the fixed source PDO delivering the maximum voltage
 `BATTERY_CHARGING_ENABLE` | Enables BC 1.2 (CDP/DCP) detection when connected to a non-USB PD power source | 1u or 0u
 `SNK_STANDBY_FET_SHUTDOWN_ENABLE` | Specifies whether the consumer power path should be disabled while PD contracts are being negotiated | 1u or 0u
 `SYS_DEEPSLEEP_ENABLE` | Enables device entry into deep sleep mode for power saving when the CPU is idle | 1u or 0u
 `APP_FW_LED_ENABLE` | Enables toggling of the user LED (LED3) based on the type of power source | 1u or 0u

<br>

### PDStack library selection

The USB Type-C Connection Manager, USB Power Delivery (USB PD) protocol layer, and USB PD device Policy Engine state machine implementations are provided in the form of pre-compiled libraries as part of the PDStack middleware library.

Multiple variants of the PDStack library with different feature sets are provided; you can choose the appropriate version based on the features required by the target application.

   - *PMG1_PD3_SNK_LITE:* Library with support for USB Type-C sink operation and USB PD Revision 3.1 messaging.

   - *PMG1_PD2_SNK_LITE:* Library with support for USB Type-C sink operation and USB PD Revision 2.0 messaging. Using this library reduces the flash (code) memory usage by the application.

   - *PMG1_PD3_SNK_EPR:* Library with support for USB Type-C sink EPR operation and USB PD Revision 3.1 messaging. This library is chosen by default.

   - *PMG1_PD3_DRP:* Library with support for USB Type-C dual-role operation and USB PD Revision 3.1 messaging.

The library of choice can be selected by editing the Makefile in the application folder and changing the value of the `COMPONENTS` variable. To disable the EPR feature, set `CY_PD_EPR_ENABLE` to 0 in `DEFINES` in the Makefile and replace the `PMG1_PD3_SNK_EPR` reference with `PMG1_PD3_SNK_LITE`. To use the PD Revision 2.0 library, replace the reference with `PMG1_PD2_SNK_LITE`.


### USB PD port configuration

Configure the properties of the USB-C port including the port role and the default response to various USB PD messages using the EZ-PD&trade; Configurator utility.

These parameters have been set to the appropriate values for a Power Delivery sink application by default. To view or change the configuration, click the **EZ-PD&trade; Configurator 1.20** item under **Tools** in the Quick Panel to launch the configurator.

**Figure 3. USB Type-C port configuration using EZ-PD&trade;  Configurator**

<img src = "images/ezpd_port_info.png" width = "800"/>

<br>

Configure the properties of the USB-C port using the *Port Information* section. Because this application supports only the USB PD sink operation, the **Port Role** must be set as **Sink**, and **DRP Toggle** must be disabled. Other parameters such as **Manufacturer Vendor ID** and **Manufacturer Product ID** can be set to desired values.

The **Source PDO** and **SCEDB Configuration** sections are not applicable to this application because only the sink operation is supported.

**Figure 4. Sink capability configuration using EZ-PD&trade; Configurator**

<img src = "images/ezpd_sink_pdo.png" width = "800"/>

<br>

The power capabilities supported by the application in the sink role are specified using the *Sink PDO* section. See the *USB Power Delivery* specification for details on how to encode the various sink capabilities. Up to seven PDOs can be added using the configurator.

**Figure 5. Extended sink capability configuration using EZ-PD&trade; Configurator**

<img src = "images/ezpd_ext_snk_cap.png" width = "800"/>

<br>

The *SKEDB* section is used to input the extended sink capabilities response that will be sent by the application when queried by the power source. See the Power Delivery specification for details on the extended sink capabilities format.

**Figure 6. EPR sink support using EZ-PD&trade; Configurator**

<img src = "images/ezpd-epr-sink-enable.png" width = "800"/>

<br>

EPR support is enabled using the *EPR Configuration* section.

**Figure 7. EPR sink capability configuration using EZ-PD&trade; Configurator**

<img src = "images/ezpd-epr-sink-pdo.png" width = "800"/>

<br>

The EPR capabilities supported by the application in the sink role are specified using the *EPR Sink PDO* section. Up to six PDOs can be added using the configurator; the maximum voltage supported is 28 V.

After the parameters have been updated as desired, save the configuration and build the application.

For quick verification of the application configurability, disable the **PD Operation** parameter under **Port Information**. When the EZ-PD&trade; PMG1 MCU device is programmed with this modification, you can see that the user LED blinks at a slower rate even when connected to a power source which supports USB Power Delivery.


### CAPSENSE&trade; configuration

In this project, the EZ-PD&trade; PMG1-S3 MCU scans a self-capacitance (CSD)-based 5-element CAPSENSE&trade; slider, and two CAPSENSE&trade; buttons for user input. The project uses the [CAPSENSE&trade; middleware](https://github.com/Infineon/capsense) (see [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) for details on selecting a middleware).

See [AN85951 – PSoC&trade; 4 and PSoC&trade; 6 MCU CAPSENSE&trade; Design Guide](https://www.infineon.com/an85951) for details on CAPSENSE&trade; features and usage.

In this application, the state of the LED is controlled based on user inputs provided using the CAPSENSE&trade; buttons and slider. Each sensor is mapped to an LED which turns ON when it is touched and turns OFF when the finger is removed. The LED mapping to the CAPSENSE&trade; sensor is provided in the operation section of this file.

The [ModusToolbox&trade; CAPSENSE&trade; Configurator tool guide](https://www.infineon.com/ModusToolboxCapSenseConfig) describes step-by-step instructions on how to configure CAPSENSE&trade; in the application. The CAPSENSE&trade; Configurator and Tuner tools can be launched from the CSD personality in the Device Configurator tool.

### Resources and settings

**Table 1. Application resources**

 Resource  | Alias/object   | Purpose 
 :-------  | :------------  | :------------------------------------
 USBPD     | PD_PORT0       | USB PD block used for PD communication
 CSD (BSP) | CYBSP_CSD      | CSD block used for CAPSENSE&trade; application
 LED (BSP) | CYBSP_USER_LED | User LED to indicate connection state
 LED (BSP) | CYBSP_LED_BTN0 | User LED to indicate touch on button 0
 LED (BSP) | CYBSP_LED_BTN1 | User LED to indicate touch on button 1
 LED (BSP) | CYBSP_LED_SLD0 | User LED to indicate touch on slider 0
 LED (BSP) | CYBSP_LED_SLD1 | User LED to indicate touch on slider 1
 LED (BSP) | CYBSP_LED_SLD2 | User LED to indicate touch on slider 2
 LED (BSP) | CYBSP_LED_SLD3 | User LED to indicate touch on slider 3
 LED (BSP) | CYBSP_LED_SLD4 | User LED to indicate touch on slider 4

<br>

### List of application files and their usage

 File                         | Purpose
 :--------------------------- | :------------------------------------ 
 *src/app/app.c & .h*                | Defines data structures and function prototypes, and implements functions to handle application-level USB Type-C and PD events
 *src/app/charger_detect.c & .h*     | Defines data structures and function prototypes, and implements functions to handle BC 1.2 charger detection
 *src/app/fault_handlers.c*          | Implements functions to handle the various faults related to USB Type-C and PD
 *src/app/pdo.c & .h*                | Defines function prototypes and implements functions to evaluate source capabilities (Power Data Object)
 *src/app/psink.c & .h*              | Defines function prototypes and implements functions for power consumer path control
 *src/app/swap.c & .h*               | Defines function prototypes and implements functions to evaluate the USB PD role swap requests
 *src/app/vdm.c & .h*                | Defines data structures, function prototypes, and implements functions to handle vendor defined messages (VDM)
 *src/system/instrumentation.c & .h* | Defines data structures and function prototypes, and implements functions to monitor CPU resource usage

<br>

## Related resources

Resources | Links
-----------|------------------
Application notes |[AN232553](https://www.infineon.com/an232553) – Getting started with EZ-PD&trade; PMG1 MCU on ModusToolbox&trade; software <br> [AN232565](https://www.infineon.com/an232565) – EZ-PD&trade; PMG1 MCU hardware design guidelines and checklist <br> [AN85951](https://www.infineon.com/an85951) – PSoC&trade; 4 and PSoC&trade; 6 MCU CAPSENSE&trade; design guide
Code examples | [Using ModusToolbox&trade; software](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [EZ-PD&trade; PMG1 MCU datasheets](https://www.infineon.com/PMG1DS)
Development kits | Select your kits from the [evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries on GitHub | [mtb-pdl-cat2](https://github.com/Infineon/mtb-pdl-cat2) – Peripheral Driver Library (PDL) and docs
Middleware on GitHub  | [pdstack](https://github.com/Infineon/pdstack) – PDStack middleware library and docs <br> [pdutils](https://github.com/Infineon/pdutils) – PDUtils middleware library and docs <br> [capsense](https://github.com/Infineon/capsense) – CAPSENSE&trade; middleware library and docs
Tools | [Eclipse IDE for ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use software and tools enabling rapid development with Infineon&reg; MCUs, covering applications from embedded sense and control to wireless and cloud-connected systems using AIROC™ Wi-Fi & Bluetooth® combo devices.

<br>

## Other resources

Infineon provides a wealth of data at www.infineon.com to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: *CE233645* - *EZ-PD&trade; PMG1 MCU: USB PD sink CAPSENSE&trade;*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
 1.1.0   | Capsense LED response fix
 2.0.0   | Added EPR feature
 3.0.0   | Updated to support ModusToolbox&trade; software v3.0 and BSPs v3.X. This version is not backward compatible with previous versions of ModusToolbox™ software
 4.0.0   | Added support to renegotiate USB-C contract based on CAPSENSEtrade; button and slider sensor input

<br>

---------------------------------------------------------

© Cypress Semiconductor Corporation, 2021-2023. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress’s patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress’s published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, WICED, ModusToolbox, PSoC, CapSense, EZ-USB, F-RAM, and Traveo are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
