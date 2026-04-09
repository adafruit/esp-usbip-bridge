BOARD ?= p4-function-ev
PORT ?=

.PHONY: build flash clean

build:
	./scripts/build-board.sh $(BOARD) build $(PORT)

flash:
	./scripts/build-board.sh $(BOARD) flash $(PORT)

clean:
	./scripts/build-board.sh $(BOARD) clean
