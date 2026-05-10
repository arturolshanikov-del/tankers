CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=c99
LDFLAGS ?= -lm

TARGET  := tankers
SRC     := main.c lodepng.c
OBJ     := $(SRC:.c=.o)

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.c lodepng.h
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET) tankers.png

clean:
	rm -f $(OBJ) $(TARGET) 01_grayscale.png 02_mask.png 03_detections.png
