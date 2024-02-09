#DEVICE_PATH=`termux-usb -l | gojq -rM '.[0]'`
DEVICE_PATH="/dev/bus/usb/002/003"

CC=clang

OUT = main
CFLAGS = -I./source -I.
SRC = diskio.c source/ff.c source/ffsystem.c source/ffunicode.c main.c
OBJ = $(SRC:%.c=%.o)

ifdef MODE
	CFLAGS += -DMODE=$(MODE)
endif

xusb: xusb.c
	$(CC) xusb.c -lusb-1.0 -o $@

.PHONY: exec_xusb
exec_xusb: xusb
	termux-usb -e ./xusb $(DEVICE_PATH)

usbtest: usbtest.c
	$(CC) usbtest.c -lusb-1.0 -o usbtest

listdevs: listdevs.c
	$(CC) listdevs.c -lusb-1.0 -o $@

u-write: main.c
	rm -f main.o
	MODE=0 make $(OUT)
	mv $(OUT) $@

u-monitor: main.c
	rm -f main.o
	MODE=1 make $(OUT)
	mv $(OUT) $@

u-touch: main.c
	rm -f main.o
	MODE=2 make $(OUT)
	mv $(OUT) $@

.PHONY: write
write: u-write
	termux-usb -e ./u-write $(DEVICE_PATH)

.PHONY: list
list:
	termux-usb -l

.PHONY: monitor
monitor: u-monitor
	termux-usb -e ./u-monitor $(DEVICE_PATH)

.PHONY: touch
touch: u-touch
	termux-usb -e ./u-touch $(DEVICE_PATH)

$(OUT): $(OBJ)
	$(CC) $(OBJ) -lusb-1.0 -o $(OUT)

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: req
req:
	termux-usb -r $(DEVICE_PATH)

clean:
	-rm -rf $(OBJ) $(OUT)

FORCE:
