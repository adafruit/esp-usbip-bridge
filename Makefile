BOARD ?= p4-function-ev

.PHONY: build flash clean

build:
	./scripts/build-board.sh $(BOARD) build

flash:
	./scripts/build-board.sh $(BOARD) flash

clean:
	./scripts/build-board.sh $(BOARD) clean
