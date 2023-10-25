//	File "outputGen.h"
//	
//  Author: Rui Ma
////////////////////////////////////////////////////////////////////////////////
#ifndef __OUTPUTGEN_H__
#define __OUTPUTGEN_H__

#include "PrimateBFUGenBase.h"

#define IO_W 64
#define RATE (0.07)

using namespace llvm;

namespace {

class outputGen : virtual public PrimateBFUGenBase{
public:
    // domain vector to store all definitions and function arguments
    int outputStartLine;
    int outputDoneLine;

    // tuple: start index; end index; end index of the second flit if the header spans across two flits, default -1
    ValueMap<Value*, std::tuple<int, int, int>> outputInstRange;
    // TODO: line, col, Inst_ptr; assume each line has only one output_header
    std::map<int, std::pair<int, Value*>> outputInstLoc;
    std::set<std::pair<std::string, int>> liveinVars;
    
    outputGen() {}

    ~outputGen(){}

    std::string generateOutputMain(int numExit) {
        std::string program =   "#include \"outputUnit.h\"\n"
                                "void outputUnit::outputUnit_cmd() {\n"
                                "    sc_uint<1> state;\n"
                                "    primate_ctrl_ou::cmd_t cmd;\n\n"
                                "    cmd_in.reset();\n"
                                "    state = 0;\n"
                                "    for (int i = 0; i < NUM_THREADS; i++) {\n"
                                "        hdr_done[i].write(0);\n"
                                "        hdr_arg[i].write(0);\n"
                                "    }\n";

        for (int i = 0; i < numExit; i++) {
            program += ("    bt" + std::to_string(i) + " = 0;\n");
        }

        program +=  "    wait();\n\n"
                    "    while (true) {\n"
                    "        if (done) {\n"
                    "            hdr_done[done_tag.read()].write(0);\n"
                    "        }\n"
                    "        bool cmd_vld = cmd_in.nb_read(cmd);\n"
                    "        if (cmd_vld) {\n"
                    "            opcode = cmd.ar_opcode;\n"
                    "            if (opcode == 0x3f) {\n"
                    "                if (cmd.ar_imm == 0) {\n"
                    "                    bt0 = cmd.ar_bits;\n";

        for (int i = 1; i < numExit; i++) {
            program += ("                } else if (cmd.ar_imm == " + std::to_string(i) + ") {\n"
                        "                    bt" + std::to_string(i) + " = cmd.ar_bits;\n");
        }

        program +=  "                }\n"
                    "            } else {\n"
                    "                hdr_arg[cmd.ar_tag].write(cmd.ar_bits);\n"
                    "                hdr_done[cmd.ar_tag].write(1);\n"
                    "            }\n"
                    "        }\n"
                    "        wait();\n"
                    "    }\n"
                    "}\n\n"
                    "void outputUnit::outputUnit_req() {\n"
                    "    sc_uint<4> state;\n"
                    "    arg_t arg;\n"
                    "    sc_uint<NUM_THREADS_LG> tag;\n"
                    "    primate_stream_512_4::payload_t pkt_in;\n\n"
                    "    bfu_rdreq.reset();\n"
                    "    state = 0;\n"
                    "    int count = 0;\n"
                    "    wait();\n\n"
                    "    while (true) {\n"
                    "        if (state == 0) {\n"
                    "            pkt_in = pkt_buf_in.peek();\n"
                    "            if (hdr_done[pkt_in.tag].read() == 1) {\n"
                    "                tag = pkt_in.tag;\n"
                    "                arg.set(hdr_arg[pkt_in.tag]);\n"
                    "                state = 1;\n"
                    "            }\n"
                    "            wait();\n"
                    "        } else {\n"
                    "            state = 0;\n"
                    "            outputUnit_req_core(";

        for (auto arg_it = liveinVars.begin(); arg_it != liveinVars.end(); arg_it++) {
            program += ("sc_biguint<" + std::to_string(arg_it->second) + "> arg." + arg_it->first + ", ");
        }
        program +=  "tag);\n"
                    "        }\n"
                    "    }\n"
                    "}\n"
                    "void outputUnit::outputUnit_rsp() {\n"
                    "    sc_uint<4> state;\n"
                    "    arg_t arg;\n"
                    "    sc_uint<NUM_THREADS_LG> tag;\n"
                    "    primate_stream_512_4::payload_t pkt_in;\n\n"
                    "    bfu_rdrsp.reset();\n"
                    "    stream_out.reset();\n"
                    "    pkt_buf_in.reset();\n"
                    "    bfu_out.reset();\n"
                    "    state = 0;\n"
                    "    done = 0;\n"
                    "    done_tag = 0;\n"
                    "    wait();\n\n"
                    "    while (true) {\n"
                    "        if (state == 0) {\n"
                    "            done = false;\n"
                    "            pkt_in = pkt_buf_in.peek();\n"
                    "            if (hdr_done[pkt_in.tag].read() == 1) {\n"
                    "                tag = pkt_in.tag;\n"
                    "                arg.set(hdr_arg[pkt_in.tag]);\n"
                    "                state = 1;\n"
                    "            }\n"
                    "            wait();\n"
                    "        } else {\n"
                    "            state = 0;\n"
                    "            done = true;\n"
                    "            done_tag = tag;\n"
                    "            outputUnit_rsp_core(";
                    
        for (auto arg_it = liveinVars.begin(); arg_it != liveinVars.end(); arg_it++) {
            program += ("sc_biguint<" + std::to_string(arg_it->second) + "> arg." + arg_it->first + ", ");
        }
        program +=  "tag);\n"
                    "        }\n"
                    "    }\n"
                    "}\n";

        return program;
    }

    void getFunctionStart(std::ifstream &profile, int &pos, int &segId) {
        bool found = false;
        pos = 0;
        segId = 0;
        std::string sourceBuf;
        int line = 1;
        int col = 1;
        found = getSource(profile, sourceBuf, line, col, profSegments[0][0], profSegments[0][1], "Output_meta");
        while (!found) {
            pos = profile.tellg();
            found = getSource(profile, sourceBuf, line, col, profSegments[segId+1][0], profSegments[segId+1][1], "Output_meta");
            segId++;
        }
        segId--;
    }

    virtual double evalFunction(std::ifstream &profile, double th, const int pos, const int segId) {
        profile.seekg(pos);
        double nt = th * avgExeCountPerInst;
        unsigned long long numStaticInst = 0;
        unsigned long long numTotalStaticInst = 0;
        unsigned long long numDynamicInst = 0;
        unsigned long long numTotalDynamicInst = 0;
        int line = profSegments[segId][0];
        int col = profSegments[segId][1];
        bool lastRemoved = false;
        bool done = false;
        std::string sourceBuf;
        // jump to Output start
        sourceBuf.clear();
        for (int i = segId; i < numProfSegments; i++) {
            if (profSegments[i][3] == 0) continue;
            bool copyEn;

            int lineStart = line;
            int colStart = col;
            done = getSource(profile, sourceBuf, line, col, profSegments[i+1][0], profSegments[i+1][1], "Output_done");

            int numStaticInstSeg = numInstPerSeg[i];
            int numDynamicInstSeg = profSegments[i][2] * numStaticInstSeg;
            numTotalStaticInst += numStaticInstSeg;
            numTotalDynamicInst += numDynamicInstSeg;

            if (profSegments[i][2] < nt) {
                // Remove the segment
                if (!lastRemoved) {
                    // Last segment is not removed
                    // std::cout << sourceBuf << std::endl;
                    if (!checkKeep(i, sourceBuf)) {
                        // Insert early exit statements
                        // outFile << "cout << \"early exit\\n\";\n";
                        lastRemoved = true;
                    } else {
                        numStaticInst += numStaticInstSeg;
                        numDynamicInst += numDynamicInstSeg;
                    }
                } else {
                    lastRemoved = true;
                }
            } else {
                // Keep the segment
                numStaticInst += numStaticInstSeg;
                numDynamicInst += numDynamicInstSeg;
                lastRemoved = false;
            }

            sourceBuf.clear();
            if (done) break;
        }

        double cost = (1.0 - (1.0 - RATE) * double(numDynamicInst) / double(numTotalDynamicInst)) * 
                (double(numStaticInst) / double(numTotalStaticInst));

        // errs() << "Static inst covered: " << numStaticInst << " over " << numTotalStaticInst <<
        //  ", Dynamic inst covered: " << numDynamicInst << " over " << numTotalDynamicInst << ", cost: " << cost << "\n";

        if (numStaticInst == 0) cost = 1000000.0;

        return cost;
    }

    std::string translateReqSource(std::string source, int lineStart, int colStart) {
        std::string program;
        int line = lineStart;
        int col = colStart;
        std::istringstream iss(source);
        std::string lineBuf;

        while (std::getline(iss, lineBuf)) {
            if (line <= outputStartLine || line > outputDoneLine) {
                line++;
                continue;
            }
            auto outputInstMeta = outputInstLoc.find(line);
            if (outputInstMeta != outputInstLoc.end()) {
                Instruction *inst = dyn_cast<Instruction>((outputInstMeta->second).second);
                std::string newCode;
                std::size_t index0;
                std::size_t index3 = lineBuf.find(";", index0);
                if ((index0 = lineBuf.find("top_intf.Output_header")) != std::string::npos) {
                    Value* srcOp = inst->getOperand(2);
                    int regId = getRegIdx(srcOp);
                    newCode = "bfu_rdreq.write(primate_bfu_req_t(tag, " + std::to_string(regId) +
                        ", " + std::to_string(regId) + "));\n";
                } else if ((index0 = lineBuf.find("top_intf.Output_2header")) != std::string::npos) {
                    Value* srcOp0 = inst->getOperand(2);
                    Value* srcOp1 = inst->getOperand(4);
                    int regId0 = getRegIdx(srcOp0);
                    int regId1 = getRegIdx(srcOp1);
                    newCode = "bfu_rdreq.write(primate_bfu_req_t(tag, " + std::to_string(regId0) +
                        ", " + std::to_string(regId1) + "));\n";
                }
                lineBuf.replace(index0, index3-index0+1, newCode);
            } else if (line == outputDoneLine) {
                std::size_t index0 = lineBuf.find("top_intf.Output_done");
                std::size_t index3 = lineBuf.find(";", index0);
                std::string newCode;
                lineBuf.replace(index0, index3-index0+1, "");
            }
            line++;
            program += lineBuf;
            if (!iss.eof()) {
                program += "\n";
            }
        }
        return program;
    }

    std::string translateRspSource(std::string source, int lineStart, int colStart) {
        std::string program;
        int line = lineStart;
        int col = colStart;
        std::istringstream iss(source);
        std::string lineBuf;

        while (std::getline(iss, lineBuf)) {
            if (line <= outputStartLine || line > outputDoneLine) {
                line++;
                continue;
            }
            auto outputInstMeta = outputInstLoc.find(line);
            if (outputInstMeta != outputInstLoc.end()) {
                Instruction *inst = dyn_cast<Instruction>((outputInstMeta->second).second);
                // inst->print(errs());
                // errs() << "\n";
                // find output call location and get variable name
                std::size_t index0;
                if ((index0 = lineBuf.find("top_intf.Output_header")) != std::string::npos) {
                    std::size_t index3 = lineBuf.find(";", index0);
                    // construct substitution
                    std::string newCode;
                    auto instRange = outputInstRange[inst];
                    int bufLength = std::get<0>(instRange);
                    int length0 = std::get<1>(instRange);
                    newCode += "hdr_data = bfu_rdrsp.read();\n";
                    if (bufLength == 0) {
                        newCode += ("out_buf = hdr_data.data0.range(" + std::to_string(length0*8-1) + ", 0);\n");
                        newCode += ("empty = " + std::to_string(IO_W-length0) + ";\n");
                    } else {
                        if (bufLength + length0 >= IO_W) {
                            newCode += ("out_buf = (hdr_data.data0.range(" + std::to_string((IO_W-bufLength)*8-1) + 
                                ", 0), out_buf.range(" + std::to_string(bufLength*8-1) + ", 0));\n");
                            newCode += ("empty = 0;\n");
                            if (bufLength + length0 > IO_W) {
                                newCode += "stream_out.write(primate_io_payload_t(out_buf, tag, 0, false));\n";
                                newCode += ("out_buf = hdr_data.data0.range(" + std::to_string(length0*8-1) +
                                    ", " + std::to_string((IO_W-bufLength)*8) + ");\n");
                                newCode += ("empty = " + std::to_string(2*IO_W-bufLength-length0) + ";\n");
                            }
                        } else {
                            newCode += ("out_buf = (hdr_data.data0.range(" + std::to_string(length0*8-1) + 
                                ", 0), out_buf.range(" + std::to_string(bufLength*8-1) + ", 0));\n");
                            newCode += ("empty = " + std::to_string(IO_W-bufLength-length0) + ";\n");
                        }
                    }
                    // substitute the input call
                    lineBuf.replace(index0, index3-index0+1, newCode);
                    line++;
                } else if ((index0 = lineBuf.find("top_intf.Output_2header")) != std::string::npos) {
                    std::size_t index3 = lineBuf.find(";", index0);
                    // construct substitution
                    std::string newCode;
                    auto instRange = outputInstRange[inst];
                    int bufLength = std::get<0>(instRange);
                    int length0 = std::get<1>(instRange);
                    int length1 = std::get<2>(instRange);
                    newCode += "hdr_data = bfu_rdrsp.read();\n";
                    if (bufLength == 0) {
                        newCode += ("out_buf = (hdr_data.data1.range(" + std::to_string(length1*8-1) +
                            ", 0), hdr_data.data0.range(" + std::to_string(length0*8-1) + ", 0));\n");
                        newCode += ("empty = " + std::to_string(IO_W-length0-length1) + ";\n");
                    } else {
                        if (bufLength + length0 >= IO_W) {
                            newCode += ("out_buf = (hdr_data.data0.range(" + std::to_string((IO_W-bufLength)*8-1) + 
                                ", 0), out_buf.range(" + std::to_string(bufLength*8-1) + ", 0));\n");
                            newCode += "stream_out.write(primate_io_payload_t(out_buf, tag, 0, false));\n";
                            if (bufLength + length0 > IO_W) {
                                newCode += ("out_buf = (hdr_data.data1.range(" + std::to_string(length1*8-1) + 
                                    ", 0), hdr_data.data0.range(" + std::to_string(length0*8-1) +
                                    ", " + std::to_string((IO_W-bufLength)*8) + "));\n");
                            } else {
                                newCode += ("out_buf = hdr_data.data1.range(" + std::to_string(length1*8-1) + 
                                    ", 0);\n");
                            }
                            newCode += ("empty = " + std::to_string(2*IO_W-bufLength-length0-length1) + ";\n");
                        } else  if (bufLength + length0 + length1 >= IO_W) {
                            newCode += ("out_buf = (hdr_data.data1.range(" + std::to_string((IO_W-length0-bufLength)*8-1) + 
                                ", 0), hdr_data.data0.range(" + std::to_string(length0*8-1) + 
                                ", 0), out_buf.range(" + std::to_string(bufLength*8-1) + ", 0));\n");
                            newCode += ("empty = 0;\n");
                            if (bufLength + length0 + length1 > IO_W) {
                                newCode += "stream_out.write(primate_io_payload_t(out_buf, tag, 0, false));\n";
                                newCode += ("out_buf = hdr_data.data1.range(" + std::to_string(length1*8-1) +
                                    ", " + std::to_string((IO_W-length0-bufLength)*8) + ");\n");
                                newCode += ("empty = " + std::to_string(2*IO_W-bufLength-length0-length1) + ";\n");
                            }
                        } else {
                            newCode += ("out_buf = (hdr_data.data1.range(" + std::to_string(length1*8-1) + 
                                ", 0), hdr_data.data0.range(" + std::to_string(length0*8-1) + 
                                ", 0), out_buf.range(" + std::to_string(bufLength*8-1) + ", 0));\n");
                            newCode += ("empty = " + std::to_string(IO_W-bufLength-length0-length1) + ";\n");
                        }
                    }
                    // substitute the input call
                    lineBuf.replace(index0, index3-index0+1, newCode);
                    line++;
                }
            } else if (line == outputDoneLine) {
                std::size_t index0 = lineBuf.find("top_intf.Output_done");
                std::size_t index3 = lineBuf.find(";", index0);
                std::string newCode;
                newCode +=  "    stream_out.write(primate_io_payload_t(out_buf, tag, empty, false));\n"
                            "    do {\n"
                            "        pkt_in = pkt_buf_in.read();\n"
                            "        stream_out.write(pkt_in);\n"
                            "    } while (!pkt_in.last);\n"
                            "    bfu_out.write(tag, bt0);\n";
                lineBuf.replace(index0, index3-index0+1, newCode);
                line++;
            } else {
                line++;
            }
            program += lineBuf;
            if (!iss.eof()) {
                program += "\n";
            }
        }

        return program;
    }

    std::string insertOutputExit(int btID) {
        std::string program;
        program = "    stream_out.write(primate_io_payload_t(out_buf, tag, empty, false));\n";
        program += "    bfu_out.write(tag, bt";
        program += (std::to_string(btID) + ");\n");
        program += "    return;\n";

        return program;
    }

    void extractOutputFunction(Function &F, BasicBlock *endBB) {
        std::ifstream srcFile(filename);
        std::ofstream outFile("outputSpec.cpp");
        // double threshold = 0.5;
        // double nt = threshold * avgExeCountPerInst;
        int pos, segId;
        getFunctionStart(srcFile, pos, segId);

        double threshold = exploreFunction(srcFile, 0.5, pos, segId);
        double nt = threshold * avgExeCountPerInst;

        std::set<BasicBlock*> BBset;
        std::string program_req;
        std::string program_rsp;

        srcFile.seekg(pos);
        int line = profSegments[segId][0];
        int col = profSegments[segId][1];
        int lineClose = profSegments[segId][0];
        int colClose = profSegments[segId][1];
        bool closeValid = false;
        int btID = 1;
        bool lastRemoved = false;
        bool done = false;
        std::string sourceBuf;
        // jump to Output start
        sourceBuf.clear();

        for (int i = segId; i < numProfSegments; i++) {
            if (profSegments[i][3] == 0) continue;
            bool copyEn;

            int lineStart = line;
            int colStart = col;
            done = getSource(srcFile, sourceBuf, line, col, profSegments[i+1][0], profSegments[i+1][1], "Output_done");
            BasicBlock* BB = getBBfromLoc(profSegments[i][0], profSegments[i][1], profSegments[i+1][0], profSegments[i+1][1]);

            if (profSegments[i][2] < nt) {
                // Remove the segment
                if (!lastRemoved) {
                    // Last segment is not removed
                    // std::cout << sourceBuf << std::endl;
                    if (!checkKeep(i, sourceBuf)) {
                        // errs() << "remove\n";
                        if (checkRegionEntry(i)) {
                            // Insert left bracket
                            program_req += "{\n";
                            program_rsp += "{\n";
                        }
                        if (!getRegionCloseLoc(profSegments[i][0], profSegments[i][1], lineClose, colClose)) {
                            errs() << "unable to find the closing location of the region\n";
                        }
                        // errs() << "lineClose: " << lineClose << ", colClose: " << colClose << "\n";
                        closeValid = true;
                        // Insert early exit statements
                        // outFile << "cout << \"early exit\\n\";\n";
                        program_rsp += insertOutputExit(btID);
                        btID++;
                        lastRemoved = true;
                        copyEn = false;
                    } else {
                        // errs() << "keep\n";
                        if (BB != NULL) BBset.insert(BB);
                        copyEn = true;
                    }
                } else {
                    if (closeValid && checkInRegion(lineClose, colClose, profSegments[i][0], 
                        profSegments[i][1], profSegments[i+1][0], profSegments[i+1][1])) {
                        closeValid = false;
                        if (sourceBuf.find("else") != std::string::npos) {
                            lastRemoved = false;
                            copyEn = true;
                        } else {
                            lastRemoved = true;
                            copyEn = false;
                        }
                    } else {
                        lastRemoved = true;
                        copyEn = false;
                    }
                }
            } else {
                // Keep the segment
                if (BB != NULL) BBset.insert(BB);
                if (lastRemoved) {
                    // Last segment is removed
                    if (checkRegionExit(i)) {
                        // Insert right bracket
                        program_req += "}\n";
                        program_rsp += "}\n";
                    }
                }
                lastRemoved = false;
                copyEn = true;
            }

            if (copyEn) {
                program_req += translateReqSource(sourceBuf, lineStart, colStart);
                program_rsp += translateRspSource(sourceBuf, lineStart, colStart);
            }
            sourceBuf.clear();
            if (done) break;
        }

        BitVector *referred = getLiveinVariables(BBset);
        initializeLiveinVars(referred);

        std::string arg_list;
        for (auto arg_it = liveinVars.begin(); arg_it != liveinVars.end(); arg_it++) {
            arg_list += ("sc_biguint<" + std::to_string(arg_it->second) + "> " + arg_it->first + ", ");
        }
        arg_list += "sc_uint<NUM_THREADS_LG> tag";

        program_req = "inline void outputUnit::outputUnit_req_core(" + arg_list + ") {\n" + program_req;
        program_req += "}\n";

        std::string tmp = "inline void outputUnit::outputUnit_rsp_core(" + arg_list + ") {\n";
        tmp +=  "    sc_biguint<512> out_buf;\n"
                "    sc_uint<8> empty;\n"
                "    primate_bfu_rsp_t hdr_data;\n"
                "    primate_stream_512_4::payload_t pkt_in;\n\n";
        program_rsp = tmp + program_rsp;
        program_rsp += "}\n";

        std::string prologue = generateOutputMain(btID);
        std::string program = prologue + "\n" + program_req + "\n" + program_rsp;

        outFile << program;

        generateOutputHeader(192, btID);
    }

    std::string generateArgStruct(int REGWIDTH) {
        std::string program;
        program += "struct arg_t {\n";
        int totalSize = 0;
        std::string setFunction;
        for (auto field = liveinVars.begin(); field != liveinVars.end(); field++) {
            program += ("    sc_biguint<" + std::to_string(field->second) + "> " + field->first + ";\n");
            setFunction += ("        " + field->first + " = bv.range(" + std::to_string(totalSize+(field->second)-1) +
            ", " + std::to_string(totalSize) + ");\n");
            totalSize += field->second;
        }
        program += ("    void set(sc_biguint<" + std::to_string(totalSize) + "> bv) {\n");
        program += setFunction;
        program +=  "    }\n"
                    "    sc_biguint<REG_WIDTH> to_uint() {\n"
                    "        sc_biguint<REG_WIDTH> val = (";
        if (totalSize < REGWIDTH) {
            program += "0, ";
        }
        std::string tmp;
        for (auto field = liveinVars.rbegin(); field != liveinVars.rend(); field++) {
            tmp += (field->first + ", ");
        }
        program += tmp.substr(0, tmp.length()-2);
        program += ");\n        return val;\n    }\n};\n";

        return program;
    }

    void generateOutputHeader(int REGWIDTH, int numExit) {
        std::ofstream outFile("output.h");
        std::string program;
        program = generateStructHeader(REGWIDTH);
        program += generateArgStruct(REGWIDTH);
        outFile << program;
        outFile << numExit << "\n";
    }

    int getOutputLengthBB(BasicBlock &BB, int start) {
        int index = start;
        int totalLength = start;
        bool hasOutputHeader = false;
        for (auto inst_it = BB.begin(); inst_it != BB.end(); inst_it++) {
            Instruction *inst = &*inst_it;
            if (isa<CallInst>(*inst)) {
                auto *tmp = dyn_cast<llvm::CallInst>(&*inst);
                Function *foo = tmp->getCalledFunction();
                if (foo->getName().contains("Output_header") || foo->getName().contains("Output_2header")) {
                    // record debug info
                    if (DILocation *Loc = inst->getDebugLoc()) {
                        unsigned line = Loc->getLine();
                        unsigned col = Loc->getColumn();
                        outputInstLoc[line] = std::make_pair(col, &*inst);
                    }
                    hasOutputHeader = true;

                    int length0 = 0;
                    int length1 = 0;
                    if (foo->getName().contains("Output_header")) {
                        Value* arg = tmp->getArgOperand(1);
                        auto* argInt = dyn_cast<ConstantInt>(arg);
                        length0 = argInt->getSExtValue();
                    } else {
                        Value* arg = tmp->getArgOperand(1);
                        auto* argInt = dyn_cast<ConstantInt>(arg);
                        length0 = argInt->getSExtValue();
                        arg = tmp->getArgOperand(3);
                        argInt = dyn_cast<ConstantInt>(arg);
                        length1 = argInt->getSExtValue();
                    }
                    // inst->print(errs());
                    // errs() << "\n index: " << index << " length: " << length;
                    outputInstRange[&*inst] = std::make_tuple(index, length0, length1);
                    totalLength = (totalLength+length0+length1 >= IO_W) ? totalLength+length0+length1-IO_W : totalLength+length0+length1;
                    // errs() << ", totalLength: " << totalLength << "\n";
                }
            }
        }
        if (hasOutputHeader) {
            return totalLength;
        }
        else {
            return -1;
        }
    }

    void inferOutputFunctions(Function &F, BasicBlock* &startBB, BasicBlock* &endBB) {
        std::vector<std::pair<BasicBlock*, int>> workstack;
        std::set<Value*> visited; // visited and contains Output_head

        BasicBlock *bb;
        if (!(bb = getBBwithFunc(F, "Output_meta", outputStartLine))) {
            errs() << "Output_meta not found!\n";
            exit(1);
        }
        startBB = bb;
        int index = getOutputLengthBB(*bb, 0);
        workstack.emplace_back(bb, index);
        if (index != -1) {
            visited.insert(bb);
        }

        if (!(endBB = getBBwithFunc(F, "Output_done", outputDoneLine))) {
            errs() << "Output_done not found!\n";
            exit(1);
        }

        while (!workstack.empty()) {
            auto bb_pair = workstack.back();
            workstack.pop_back();
            
            bb = bb_pair.first;
            index = bb_pair.second;
            for (auto succ = succ_begin(bb), succEnd = succ_end(bb); succ != succEnd; ++succ) {
                BasicBlock* bb_succ = *succ;
                int newIndex = getOutputLengthBB(*bb_succ, index);
                if (newIndex != -1) {
                    if (visited.find(&*bb_succ) != visited.end()) {
                        printBasicBlock(*bb_succ);
                        errs() << "Error: Output_header already visited in this basic block\n";
                        exit(1);
                    }
                    visited.insert(bb_succ);
                    if (bb_succ != endBB) {
                        workstack.emplace_back(bb_succ, newIndex);
                    }
                } else if (bb_succ != endBB) {
                    workstack.emplace_back(bb_succ, index);
                }
            }
        }
    }

    BitVector* getLiveinVariables(std::set<BasicBlock*> &BBset) {
        BitVector* referred = new BitVector(domainSize, false);
        for (auto bb_it = BBset.begin(); bb_it != BBset.end(); bb_it++) {
            BitVector *liveins = (*in)[*bb_it];
            BitVector *liveouts = (*out)[*bb_it];
            (*liveins) &= ((*liveouts).flip());
            (*referred) |= (*liveins);
        }
        return referred;
    }

    void initializeLiveinVars(BitVector* referred) {
        for (int i=0; i < referred->size(); i++) {
            if ( (*referred)[i] ) {
                Value *var = (*bvIndexToInstrArg)[i];
                bool isInt = false;
                int width = 0;
                if (isa<AllocaInst>(*var)) {
                    auto *inst = dyn_cast<AllocaInst>(var);
                    Type *varType = inst->getAllocatedType();
                    if (isa<IntegerType>(*varType)) {
                        isInt = true;
                        auto* varIntType = dyn_cast<IntegerType>(varType);
                        width = varIntType->getBitWidth();
                    }
                }
                if (varNameMap.find(var) != varNameMap.end()) {
                    std::string name = varNameMap[var];
                    if (isInt) {
                        liveinVars.insert(std::make_pair(name, width));
                    }
                }
            }
        }
    }

    int getRegIdx(Value* var) {
        Value *rootVar = (*aliasMap)[var];
        if (varRegMap.find(rootVar) != varRegMap.end()) {
            return varRegMap[rootVar];
        } else {
            errs() << "Error: Can't get the register index of the variable.\n";
            return 0;
        }
    }

};
}

#endif

