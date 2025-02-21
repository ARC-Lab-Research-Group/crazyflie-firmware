## Copy this file to config.mk and modify to get you personal build configuration

## Weight of the Crazyflie, including decks. The default setting is a Crazyflie 2.X without decks.
#CFLAGS += -DCF_MASS=0.032f # in kg (MOCAP Deck)
#CFLAGS += -DCF_MASS=0.037f # in kg (AI Deck + MOCAP Deck)

## Enable the AI Deck CBF-QP for controller_lqr
#CFLAGS += -DCBF_TYPE_POS
#CFLAGS += -DCBF_TYPE_EUL
#CFLAGS += -DCBF_TYPE_REF
#CFLAGS += -DAI_CBF_DEBUG
#CFLAGS += -DCBF_ITERS

# Set to 1 to add OSQP to firmware compilation
#COMPILE_OSQP := 1

# Apply Integral Action to altitude control?
#CFLAGS += -DLQR_ALT_PID

# Mistune the D6LQR controller by reducing KD6 by a factor of:
#CFLAGS += -DD6LQR_MISTUNE=0.05f

# Force a default estimator with the line below
#ESTIMATOR=kalman

# Force a default controller with the line below.
#CONTROLLER=LQR

## Force device type string
# CFLAGS += -DDEVICE_TYPE_STRING_FORCE="CF20"

## Force a sensor implementation to be used
# SENSORS=bosch

## Set CRTP link to E-SKY receiver
# CFLAGS += -DUSE_ESKYLINK

## Redirect the console output to the UART
# CFLAGS += -DDEBUG_PRINT_ON_UART

## Redirect the console output to JLINK (using SEGGER RTT)
# DEBUG_PRINT_ON_SEGGER_RTT = 1

## Load a deck driver that has no OW memory
# CFLAGS += -DDECK_FORCE=bcBuzzer

## Load multiple deck drivers that has no OW memory
# CFLAGS += -DDECK_FORCE=bcBuzzer:bcLedRing

## Enable biq quad deck features
# CFLAGS += -DENABLE_BQ_DECK
# CFLAGS += -DBQ_DECK_ENABLE_PM
# CFLAGS += -DBQ_DECK_ENABLE_OSD

## Use morse when flashing the LED to indicate that the Crazyflie is calibrated
# CFLAGS += -DCALIBRATED_LED_MORSE

## Disable LEDs from turning on/flashing
# CFLAGS += -DTURN_OFF_LEDS

## Set the default LED Ring effect (if not set, effect 6 will be used)
# CFLAGS += -DLEDRING_DEFAULT_EFFECT=0

## Set LED Rings to use less LEDs
# CFLAGS += -DLED_RING_NBR_LEDS=6

## Set LED Rings to use less more LEDs (only if board is modified)
# CFLAGS += -DLED_RING_NBR_LEDS=24

## Do not send CRTP messages when parameter values are updated
# CFLAGS += -DSILENT_PARAM_UPDATES

## Turn on monitoring of queue usages
# CFLAGS += -DDEBUG_QUEUE_MONITOR

## Automatically reboot to bootloader before flashing
# CLOAD_CMDS = -w radio://0/100/2M/E7E7E7E7E7

## Set number of anchor in LocoPositioningSystem
# CFLAGS += -DLOCODECK_NR_OF_ANCHORS=8

## Set alternative pins for LOCO deck (IRQ=IO_2, RESET=IO_3, default are RX1 and TX1)
# CFLAGS += -DLOCODECK_USE_ALT_PINS

## Set other pin for reset on the LOCO deck. Only works when LOCODECK_USE_ALT_PINS is set.
# For instance useful with Loco + Lighhouse + Flow decks where IO_3 collides with the Flow deck
# CFLAGS += -DLOCODECK_ALT_PIN_RESET=DECK_GPIO_IO4

## Disable Low Interference Mode when using Loco Deck
# CFLAGS += -DLOCODECK_NO_LOW_INTERFERENCE

## Low interference communication
# Set the 'low interference' 2.4GHz TX power. This power is set when the loco deck is initialized
# Possible power are: +4, 0, -4, -8, -12, -16, -20, -30
# CFLAGS += -DPLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM="(-12)"

## Set alternative pins for uSD-deck. Patch soldering required (CS->RX2(PA3), SCLK->TX1(PC10), MISO->RX1(PC11), MOSI->IO_4(PC12))
# CFLAGS += -DUSDDECK_USE_ALT_PINS_AND_SPI

## Use J-Link as Debugger/flasher
# OPENOCD_INTERFACE ?= interface/jlink.cfg
# OPENOCD_TARGET    ?= target/stm32f4x.cfg
# OPENOCD_CMDS      ?= -c "transport select swd"

## LPS settings ----------------------------------------------------
## Set operation mode of the LPS system (auto is default)
## TWR
# CFLAGS += -DLPS_TWR_ENABLE=1
## TDoA2
# LPS_TDOA_ENABLE=1
## TDoA3
# LPS_TDOA3_ENABLE=1

## TDoA 3 - experimental
# Enable 2D positioning. The value (1.2) is the height that the tag will move at. Only use in TDoA 3
# CFLAGS += -DLPS_2D_POSITION_HEIGHT=1.2

## Enable longer range (lower bit rate). Only use in TDoA 3
# Note: Anchors must also be built with this flag
# CFLAGS += -DLPS_LONGER_RANGE

## Full LPS TX power.
# CFLAGS += -DLPS_FULL_TX_POWER

## SDCard test configuration ------------------------------------
# FATFS_DISKIO_TESTS  = 1	# Set to 1 to enable FatFS diskio function tests. Erases card.

## To build with lighthouse support
# CFLAGS += -DDISABLE_LIGHTHOUSE_DRIVER=0

## Force lighthouse V1 or V2 type system
# Lighthouse V1
# CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=1
# Lighthouse V2
# CFLAGS += -DLIGHTHOUSE_FORCE_TYPE=2

## Brushless handling
# Start disarmed, needs to be armed before being able to fly
# CFLAGS += -DSTART_DISARMED
# IDLE motor drive when armed, 0 = 0%, 65535 = 100% (the motors runs as long as the Crazyflie is armed)
# CFLAGS += -DDEFAULT_IDLE_THRUST=5000

## Use Plane Distance measurements on Multiranger
#CFLAGS += -DPLANE_DISTANCE
