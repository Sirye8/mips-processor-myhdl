# MIPS Processor Simulator

![Python](https://img.shields.io/badge/Python-3.8%2B-blue) ![MyHDL](https://img.shields.io/badge/MyHDL-0.11-green) ![MyHDLPeek](https://img.shields.io/badge/MyHDLPeek-0.1.0-yellowgreen) 

A single-cycle MIPS processor simulator implemented in Python using **MyHDL**, supporting core instructions (`ADD`, `SUB`, `ADDI`, `BEQ`, `LW`, `SW`, `XOR`) with register file, control unit, and data memory operations.

## Features
- ✅ R-type/I-type instruction support  
- 🖥️ Full pipeline: Fetch, Decode, Execute, Memory, Writeback  
- 🔧 Integrated components: Control Unit, ALU, Sign Extender, Register File (32 registers), Data Memory  
- 📊 MyHDLPeek for real-time waveform visualization

## Installation
```bash
pip install myhdl myhdlpeek
```

## Usage
1. Clone this repository  
2. Run the simulator:
```python
python mips_processor.py
```
3. View generated waveforms (`*.vcd` files) using `myhdlpeek` and `matplotlib`.