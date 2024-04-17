# Makefile for VHDL project with GTKWave
.PHONY: format all compile simulate view clean lsp help

# Compiler and flags
VHDL_COMPILER = ghdl

# Compiler flags
VHDL_FLAGS = --std=08

# Get all VHDL files in the current directory
VHDL_FILES = $(wildcard *.vhdl)

# Extract testbench file name without extension
TESTBENCH = $(basename $(wildcard *_tb.vhdl))

# Testbench and waveform file
WAVEFORM_FILE = $(TESTBENCH).ghw

# Default target
help:
	@echo "Usage: make [target]"
	@echo "Available targets:"
	@echo "  all       - Compile, simulate and view the waveform"
	@echo "  compile   - Compile the VHDL files"
	@echo "  simulate  - Simulate the testbench"
	@echo "  view      - View the waveform using GTKWave"
	@echo "  format    - Format the VHDL files"
	@echo "  lsp       - Generate Language Server Protocol configuration"
	@echo "  clean     - Clean generated files"
	@echo "  help      - Display this help message"

# All targets
all: compile simulate view


# Compile VHDL files
compile:
	$(VHDL_COMPILER) -a $(VHDL_FLAGS) $(VHDL_FILES)

# Simulate the testbench
# Remember to add assertion or finish to the testbench!
# E.g. "assert false report "End of test" severity note; wait;"
# E.g. "report "End of test"; finish;"
simulate:
	$(VHDL_COMPILER) -r $(VHDL_FLAGS) $(TESTBENCH) --wave=$(WAVEFORM_FILE)

# View the waveform using GTKWave
view:
	gtkwave $(WAVEFORM_FILE)

format:
	for file in $(filter-out %_tb.vhdl,$(VHDL_FILES)); do \
		ghdl --format $(VHDL_FLAGS) $$file > $${file%.*}_temp.vhdl && mv $${file%.*}_temp.vhdl $$file; \
	done
	for file in $(filter %_tb.vhdl,$(VHDL_FILES)); do \
		ghdl --format $(VHDL_FLAGS) $$file > $${file%.*}_temp.vhdl && mv $${file%.*}_temp.vhdl $$file; \
	done

# Clean generated files
clean:
	rm -f *.ghw *.cf

lsp:
	@echo "[libraries]" > vhdl_ls.toml
	@echo "lib.files = [" >> vhdl_ls.toml
	@for file in $(VHDL_FILES); do \
		echo "  \"$$file\"," >> vhdl_ls.toml; \
	done
	@echo "]" >> vhdl_ls.toml

