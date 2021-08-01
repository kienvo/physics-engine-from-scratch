
TARGET = freefall.elf
CPPFLAGS += #-g -Wall -std=gnu++17

.PHONY: all clean run check

all: $(TARGET)


$(TARGET): main.o
	g++ $< -o $@ -lsfml-graphics -lsfml-window -lsfml-system -g
clean:
	rm -rf ./$(TARGET) *.o *.elf
run: $(TARGET)
	./$<
check:
	valgrind ./$(TARGET) 2> check.txt