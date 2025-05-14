from myhdl import *
from myhdlpeek.myhdl import Peeker
import matplotlib.pyplot as plt

with open('Instructions.txt', 'r', encoding='utf-8') as f:
    file_contents = f.read()
    print(file_contents)

listinst = file_contents.splitlines()
intbvlist = []
for item in listinst:
    item = item.lstrip('\ufeff')  # Strip BOM
    value = int(item, 2)          # Convert binary string to int
    intbv_signal = intbv(value)[32:]  # Keep as an unsigned 32-bit value
    intbvlist.append(intbv_signal)
print(intbvlist)

# FETCH
@block
def fetch(clk, pc, inst, Branch, zero, offset):
    @always(clk.posedge)
    def fetch_logic():
        if pc < len(intbvlist):
            inst.next = intbvlist[int(pc)]
            if Branch and zero:
                pc.next = pc + 1 + offset
            else:
                pc.next = pc + 1
        else:
            inst.next = 0
    return fetch_logic

# DECODE

@block
def decode(inst, opcode, funct, rs, rt, rd, im,clk):
    @always(clk.posedge)
    def logic():
        opcode.next = inst[32:26]
        rs.next = inst[26:21]
        rt.next = inst[21:16]
        rd.next = inst[16:11]
        if inst[32:26] == 0:
            im.next = 0
        else:
            im.next = inst[16:0]
        funct.next = inst[6:0]
    return logic


# Control Unit
@block
def ControlUnit(opcode, funct,
                RegDst, RegWrite, ALUSrc, ALUOp,
                MemWrite, MemRead, MemToReg, Branch):

    @always_comb
    def logic():
        # Default values
        RegDst.next = 0
        RegWrite.next = 0
        ALUSrc.next = 0
        ALUOp.next = 0
        MemWrite.next = 0
        MemRead.next = 0
        MemToReg.next = 0
        Branch.next = 0

        if opcode == 0b000000:  # R-Type
            RegDst.next = 1
            RegWrite.next = 1
            if funct == 0b100000:       # ADD
                ALUOp.next = 0b010
            elif funct == 0b100010:     # SUB
                ALUOp.next = 0b110
            elif funct == 0b110110:     # AND
                ALUOp.next = 0b000
            elif funct == 0b110111:     # OR
                ALUOp.next = 0b001
            elif funct == 0b111000:     # XOR
                ALUOp.next = 0b011

        elif opcode == 0b001000:  # ADDI
            RegDst.next = 0
            RegWrite.next = 1
            ALUSrc.next = 1
            ALUOp.next = 0b010

        elif opcode == 0b100100:  # SUBI
            RegDst.next = 0
            RegWrite.next = 1
            ALUSrc.next = 1
            ALUOp.next = 0b110

        elif opcode == 0b100011:  # LW
            RegDst.next = 0
            RegWrite.next = 1
            ALUSrc.next = 1
            ALUOp.next = 0b010
            MemRead.next = 1
            MemToReg.next = 1

        elif opcode == 0b101011:  # SW
            ALUSrc.next = 1
            ALUOp.next = 0b010
            MemWrite.next = 1

        elif opcode == 0b000100:  # BEQ
            ALUSrc.next = 0
            ALUOp.next = 0b110
            Branch.next = 1

    return logic


# sign-extend

@block
def sign_extend(im, newim):
    @always_comb
    def extend():
        if im[15] == 1:
            newim.next = intbv(int(im) | 0xFFFF0000)[32:]
        else:
            newim.next = intbv(int(im) & 0x0000FFFF)[32:]
    return extend


# mux
@block
def mux(a, b, sel, out):
  @always_comb
  def logic():
    out.next = a if sel == 0 else b
  return logic

# Register file
from myhdl import block, always_comb, always, Signal, intbv
@block
def reg_file(clk, RegWrite, rs, rt, ReadData1, ReadData2):
  initial_values =[4,5,3,6,7,5,8,7,1,23,55,4,5,65,73,45,16,7,3,22,11,55,15,76,5,3,45,12,23,12,6,0]
  regfile = [Signal(intbv(val)[32:]) for val in initial_values]
  @always_comb
  def read_logic():
      ReadData1.next = regfile[int(rs)]
      ReadData2.next = regfile[int(rt)]
  return read_logic

#ALU
@block
def ALU(A, B, ALUOp, result, ZERO):
    @always_comb
    def alu_logic():
        if ALUOp == 0b000:  # AND
            result.next = A & B
        elif ALUOp == 0b001:  # OR
            result.next = A | B
        elif ALUOp == 0b010:  # ADD
            result.next = A + B
        elif ALUOp == 0b110:  # SUB
            result.next = A - B
        elif ALUOp == 0b011:  # XOR
            result.next = A ^ B
        else:
            result.next = 0

        ZERO.next = int(result == 0)
    return alu_logic

@block
def DataMemory(clk, address, write_data, read_data, mem_write, mem_read):
    mem = [Signal(intbv(0)[32:]) for _ in range(64)]

    @always(clk.posedge)
    def mem_logic():
        addr = int(address[6:])
        if mem_write:
            mem[addr].next = write_data
        if mem_read:
            read_data.next = mem[addr]
    return mem_logic

#shift left
@block
def ShiftLeft2(inp, out):
    @always_comb
    def logic():
        out.next = inp << 2
    return logic

#AND gate
@block
def AndGate(a, b, out):
    @always_comb
    def logic():
        out.next = a and b
    return logic

# jumpAdder
@block
def JumpAdder(pc, offset, branch_addr):
    @always_comb
    def logic():
        branch_addr.next = pc + offset
    return logic

#4th mux
@block
def fourth_mux (sel, a, b, out):
  @always(sel)
  def logic():
    out.next = a[5:] if sel == 0 else b[5:]
  return logic

#PC increment
@block
def PCIncrement(pc_in, pc_out):
    @always_comb
    def logic():
        pc_out.next = pc_in + 4
    return logic



### Simulation


@block
def top():
    clk = Signal(bool(0))
    reset = ResetSignal(1, active=1, isasync=True)
    pc = Signal(intbv(0)[32:])
    inst = Signal(intbv(0)[32:])
    opcode = Signal(intbv(0)[6:])
    funct = Signal(intbv(0)[6:])
    rs = Signal(intbv(0)[5:])
    rt = Signal(intbv(0)[5:])
    rd = Signal(intbv(0)[5:])
    im = Signal(intbv(0)[16:])
    newim = Signal(intbv(0)[32:])
    ReadData1 = Signal(intbv(0)[32:])
    ReadData2 = Signal(intbv(0)[32:])
    WriteData = Signal(intbv(0, min=-2**32, max=2**32))
    WriteReg = Signal(intbv(0)[5:])
    alu_input2 = Signal(intbv(0)[32:])
    alu_result = Signal(intbv(0, min=-2**32, max=2**32))
    zero = Signal(intbv(0)[1:])
    read_data_mem = Signal(intbv(0)[32:])
    shifted_branch_address = Signal(intbv(0)[32:])
    branch_taken = Signal(bool(0))
    inc_pc = Signal(intbv(0)[32:])
    jump_adder_result = Signal(intbv(0)[32:])
    new_pc = Signal(intbv(0)[5:])
    and_out = Signal(bool(0))

    RegDst = Signal(intbv(0)[1:])
    RegWrite = Signal(intbv(0)[1:])
    ALUSrc = Signal(intbv(0)[1:])
    ALUOp = Signal(intbv(0)[3:])
    MemWrite = Signal(intbv(0)[1:])
    MemRead = Signal(intbv(0)[1:])
    MemToReg = Signal(intbv(0)[1:])
    Branch = Signal(intbv(0)[1:])

    # Peekers
    Peeker(opcode, 'opcode')
    Peeker(pc, 'pc')
    Peeker(inst, 'inst')
    Peeker(ReadData1, 'readData1')
    Peeker(ReadData2, 'readData2')
    Peeker(WriteReg, 'writeReg')
    Peeker(WriteData, 'writeDataReg')
    Peeker(alu_result, 'ALUResult')
    Peeker(MemRead, 'MemReadBool')
    Peeker(MemWrite, 'MemWriteBool')
    Peeker(read_data_mem, 'ReadData')
    Peeker(zero, 'Zero')
    Peeker(clk, 'clk')
    Peeker(newim, 'newim')
    Peeker(rd, 'rd')
    Peeker(rs, 'rs')
    Peeker(rt, 'rt')
    Peeker(alu_input2, 'writeDataMemory')
    Peeker(ALUSrc, 'ALUSrc')
    Peeker(Branch, 'Branch')
    Peeker(jump_adder_result,"jumpadderresult")
    Peeker(shifted_branch_address,"shiftedim")
    Peeker(and_out,"andingresult")

    # Block Instantiations
    fetch_inst = fetch(clk, pc, inst, Branch, zero, newim)
    decode_inst = decode(inst, opcode, funct, rs, rt, rd, im,clk)
    control_inst = ControlUnit(opcode, funct, RegDst, RegWrite, ALUSrc, ALUOp, MemWrite, MemRead, MemToReg, Branch)
    sign_ext_inst = sign_extend(im, newim)
    mux1_inst = mux(rt, rd, RegDst, WriteReg)
    reg_read_inst = reg_file(clk, RegWrite, rs, rt, ReadData1, ReadData2)
    mux2_inst = mux(ReadData2, newim, ALUSrc, alu_input2)
    alu_inst = ALU(ReadData1, alu_input2, ALUOp, alu_result, zero)
    mux3_inst = mux(alu_result, read_data_mem, MemToReg, WriteData)
    data_mem_inst = DataMemory(clk, alu_result, ReadData2, read_data_mem, MemWrite, MemRead)
    anding_inst = AndGate(zero, Branch, and_out)
    shift_left_inst = ShiftLeft2(newim, shifted_branch_address)
    pc_increment_inst = PCIncrement(pc, inc_pc)
    jump_adder_inst = JumpAdder(inc_pc, shifted_branch_address, jump_adder_result)
    mux4_inst = fourth_mux(and_out, inc_pc, jump_adder_result, pc)





    @instance
    def clkgen():
        for _ in range(30):
            yield delay(10)
            clk.next = not clk

    return (fetch_inst, decode_inst, control_inst, sign_ext_inst, mux1_inst, reg_read_inst, mux2_inst,  alu_inst, mux3_inst, data_mem_inst,
            anding_inst, shift_left_inst, pc_increment_inst, jump_adder_inst, mux4_inst, clkgen)

sim = Simulation(top(), *Peeker.instances())
sim.run()
Peeker.show_waveforms(width=30, slope=0)
plt.show()
Peeker.clear()