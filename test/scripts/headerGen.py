import re

def removeComments(text):
    def replacer(match):
        s = match.group(0)
        if s.startswith('/'):
            return " " # note: a space and not an empty string
        else:
            return s
    pattern = re.compile(
        r'//.*?$|/\*.*?\*/|\'(?:\\.|[^\\\'])*\'|"(?:\\.|[^\\"])*"',
        re.DOTALL | re.MULTILINE
    )
    return re.sub(pattern, replacer, text)

def generateHeader(inFile, outFile):
    with open(inFile, "r") as f_src, open("tmp.h", "r") as f_struct, open(outFile, "w") as f_dst:
        text = f_src.read()
        text = removeComments(text)
        lines = text.split("\n")
        f_dst.write("#include \"common.h\"\n")

        state = 0
        for line in lines:
            if state == 0:
                if line.startswith("#define"):
                    f_dst.write(line + "\n")
                    if line[-1] == '\\':
                        state = 1
            elif state == 1:
                f_dst.write(line + "\n")
                if line[-1] != '\\':
                    state = 0
        f_dst.write("\n")

        lines = f_struct.read().splitlines()
        lastline = lines[-1]
        print(lastline)
        for i in range(len(lines)-1):
            f_dst.write(lines[i])
            f_dst.write("\n")

        text = """SC_MODULE(inputUnit) {
    sc_in<bool> i_clk;
    sc_in<bool> i_rst;

    sc_uint<NUM_THREADS_LG> tag;
    sc_uint<OPCODE_WIDTH> opcode;\n"""

        for i in range(int(lastline)):
            text += ("    sc_uint<IP_WIDTH> bt" + str(i) + ";\n")

        text += "\n"
        text += """    primate_stream_512_4::in          stream_in;
    primate_ctrl_iu::slave            cmd_in;
    primate_bfu_iu::master            bfu_out;
    primate_stream_512_4::out         pkt_buf_out;

    void inputUnit_main();
    inline void inputUnit_core();

    SC_CTOR(inputUnit) {
        SC_CTHREAD(inputUnit_main, i_clk.pos());
        reset_signal_is(i_rst, true);
    };
};\n"""
        f_dst.write(text)

if __name__ == '__main__':
    hdrFilename = "primate.h"
    outFilename = "inputUnit.h"

    generateHeader(hdrFilename, outFilename)


        