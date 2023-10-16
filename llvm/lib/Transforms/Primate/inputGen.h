//	File "inputGen.h"
//	
//	To perform backward and forward analysis of the source program
////////////////////////////////////////////////////////////////////////////////
#ifndef __INPUTGEN_H__
#define __INPUTGEN_H__

#include "PrimateBFUGenBase.h"

using namespace llvm;

#define RATE (0.05)

namespace {

class inputGen : virtual public PrimateBFUGenBase{
public:
    // domain vector to store all definitions and function arguments
    int inputDoneLine;

    // tuple: start index; end index; end index of the second flit if the header spans across two flits, default -1
    ValueMap<Value*, std::tuple<int, int, int>> inputInstRange;
    // TODO: line, col, Inst_ptr; assume each line has only one Input_header
    std::map<int, std::pair<int, Value*>> inputInstLoc;
    
    inputGen() {}

    ~inputGen(){}

    std::string generateInputMain(int numExit) {
        std::string program =   "#include \"inputUnit.h\"\n"
                                "void inputUnit::inputUnit_main() {\n"
                                "    primate_ctrl_iu::cmd_t cmd;\n\n"
                                "    stream_in.reset();\n"
                                "    cmd_in.reset();\n"
                                "    pkt_buf_out.reset();\n"
                                "    bfu_out.reset();\n";

        for (int i = 0; i < numExit; i++) {
            program += ("    bt" + std::to_string(i) + " = 0;\n");
        }

        program +=  "    wait();\n"
                    "    while (true) {\n"
                    "        cmd = cmd_in.read();\n"
                    "        tag = cmd.ar_tag;\n"
                    "        opcode = cmd.ar_opcode;\n"
                    "        if (opcode == 0x3f) {\n"
                    "            if (cmd.ar_imm == 0) {\n"
                    "                bt0 = cmd.ar_bits;\n";

        for (int i = 1; i < numExit; i++) {
            program += ("            } else if (cmd.ar_imm == " + std::to_string(i) + ") {\n"
                        "                bt" + std::to_string(i) + " = cmd.ar_bits;\n");
        }

        program +=  "            }\n"
                    "            wait();\n"
                    "        } else {\n"
                    "            inputUnit_core();\n"
                    "        }\n"
                    "    }\n"
                    "}\n\n";

        return program;
    }

    double evalFunction(std::ifstream &profile, double th, const int pos, const int segId) {
        profile.seekg(0);
        double nt = th * avgExeCountPerInst;
        unsigned long long numStaticInst = 0;
        unsigned long long numTotalStaticInst = 0;
        unsigned long long numDynamicInst = 0;
        unsigned long long numTotalDynamicInst = 0;
        int line = 1;
        int col = 1;
        bool lastRemoved = false;
        bool done;
        std::string sourceBuf;
        getSource(profile, sourceBuf, line, col, profSegments[0][0], profSegments[0][1]+1, "Input_done");
        col++; // skip the first "{"
        sourceBuf.clear();
        for (int i = 0; i < numProfSegments; i++) {
            if (profSegments[i][3] == 0) continue;
            bool copyEn;

            int lineStart = line;
            int colStart = col;
            done = getSource(profile, sourceBuf, line, col, profSegments[i+1][0], profSegments[i+1][1], "Input_done");

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

    double exploreFunction(std::ifstream &profile, const double init_th, const int pos, const int segId) {
        double th_old = init_th;
        int count = 100;

        // Simulated annealing algorithm
        double cost_old = evalFunction(profile, th_old, pos, segId);
        double th_best = th_old;
        double cost_best = cost_old;

        std::random_device rd;
        std::mt19937_64 g(rd());

        std::uniform_real_distribution<double> rf(0, 1);

        for (; count > 0; --count) {
            double th_new = nf<decltype(g)>(th_old, g);
            double cost_new = evalFunction(profile, th_new, pos, segId);

            // errs() << "new th: " << th_new << ", new cost: " << cost_new << ". ";

            if (cost_new < cost_best) {
                th_best = th_new;
                cost_best = cost_new;
            }

            if (cost_new < cost_old || std::exp((cost_old - cost_new) / count) > rf(g)) {
                // errs() << "move\n";
                th_old = th_new;
                cost_old = cost_new;
            }
        }
        // errs() << "\n";

        return th_best;
    }

    std::string translateSource(std::string source, int lineStart, int colStart) {
        std::string program;
        int line = lineStart;
        int col = colStart;
        std::istringstream iss(source);
        std::string lineBuf;
        while (std::getline(iss, lineBuf)) {
            auto inputInstMeta = inputInstLoc.find(line);
            if (inputInstMeta != inputInstLoc.end()) {
                Instruction *inst = dyn_cast<Instruction>((inputInstMeta->second).second);
                // inst->print(errs());
                // errs() << "\n";
                // find input call location and get variable name
                std::size_t index0 = lineBuf.find("top_intf.Input_header");
                std::size_t index1 = lineBuf.find(",", index0);
                std::size_t index2 = lineBuf.find(")", index0);
                std::size_t index3 = lineBuf.find(";", index0);
                std::string varName = lineBuf.substr(index1+1, index2-index1-1);
                varName.erase(std::remove_if(varName.begin(), varName.end(), ::isspace), varName.end());
                // construct substitution
                std::string newCode;
                auto instRange = inputInstRange[inst];
                int buf0Start = std::get<0>(instRange);
                int buf0End = std::get<1>(instRange);
                int buf1End = std::get<2>(instRange);
                if (buf0Start == 0) {
                    newCode += "payload = stream_in.read();\n";
                    newCode += (varName + ".set(payload.data.range(" + std::to_string((buf0End+1)*8-1) + ", 0));\n");
                    newCode += ("in_data_buf = payload.data;\nlast_buf = payload.last;\npkt_empty = " + 
                        std::to_string(buf0End+1) + ";\npkt_data_buf = payload.data.range(" + 
                        std::to_string(IO_W*8-1) + ", " + std::to_string((buf0End+1)*8) + ");");
                } else if (buf1End != -1) {
                    newCode += "payload = stream_in.read();\n";
                    newCode += (varName + ".set((payload.data.range(" + std::to_string((buf1End+1)*8-1) + 
                        ", 0), in_data_buf.range(" + std::to_string((buf0End+1)*8-1) + ", " + std::to_string(buf0Start*8) + ")));\n");
                    newCode += ("in_data_buf = payload.data;\nlast_buf = payload.last;\npkt_empty = " + 
                        std::to_string(buf1End+1) + ";\npkt_data_buf = payload.data.range(" + std::to_string(IO_W*8-1) +
                        ", " + std::to_string((buf1End+1)*8) + ");");
                } else {
                    newCode += (varName + ".set(in_data_buf.range(" + std::to_string((buf0End+1)*8-1) + ", " + 
                        std::to_string(buf0Start*8) + "));\n");
                    newCode += ("pkt_empty = " + std::to_string(buf0End+1) + ";\npkt_data_buf = payload.data.range(" + 
                        std::to_string(IO_W*8-1) + ", " + std::to_string((buf0End+1)*8) + ");");
                }

                // substitute the input call
                lineBuf.replace(index0, index3-index0+1, newCode);
                line++;
            } else if (line == inputDoneLine) {
                std::size_t index0 = lineBuf.find("top_intf.Input_done");
                std::size_t index3 = lineBuf.find(";", index0);
                std::string newCode;
                newCode += ("payload.data = pkt_data_buf;\n"
                        "    payload.empty = pkt_empty;\n"
                        "    payload.last = last_buf;\n"
                        "    pkt_buf_out.write(payload);\n"
                        "    while (!last_buf) {\n"
                        "        payload = stream_in.read();\n"
                        "        last_buf = payload.last;\n"
                        "        pkt_buf_out.write(payload);\n"
                        "    }");
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

    void extractInputFunction(Function &F, BasicBlock *endBB) {
        std::ifstream srcFile(filename);
        std::ofstream outFile("inputSpec.cpp");
        // double threshold = 0.5;
        // double nt = threshold * avgExeCountPerInst;

        double threshold = exploreFunction(srcFile, 0.5, 0, 0);
        double nt = threshold * avgExeCountPerInst;
        srcFile.seekg(0);

        std::set<BasicBlock*> BBset;
        std::string program;

        program +=  "void inputUnit::inputUnit_core() {\n"
                    "    primate_stream_512_4::payload_t payload;\n"
                    "    sc_biguint<512> in_data_buf;\n"
                    "    sc_biguint<512> pkt_data_buf;\n"
                    "    bool last_buf;\n"
                    "    sc_uint<8> pkt_empty;\n";

        int line = 1;
        int col = 1;
        int btID = 1;
        bool lastRemoved = false;
        bool done;
        std::string sourceBuf;
        getSource(srcFile, sourceBuf, line, col, profSegments[0][0], profSegments[0][1]+1, "Input_done");
        col++; // skip the first "{"
        sourceBuf.clear();
        for (int i = 0; i < numProfSegments; i++) {
            if (profSegments[i][3] == 0) continue;
            bool copyEn;

            int lineStart = line;
            int colStart = col;
            done = getSource(srcFile, sourceBuf, line, col, profSegments[i+1][0], profSegments[i+1][1], "Input_done");
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
                            program += "{\n";
                        }
                        // Insert early exit statements
                        // outFile << "cout << \"early exit\\n\";\n";
                        program += insertInputExit(BBset, &(F.getEntryBlock()), BB, endBB, btID);
                        btID++;
                        lastRemoved = true;
                        copyEn = false;
                    } else {
                        // errs() << "keep\n";
                        if (BB != NULL) BBset.insert(BB);
                        copyEn = true;
                    }
                } else {
                    lastRemoved = true;
                    copyEn = false;
                }
            } else {
                // Keep the segment
                if (BB != NULL) BBset.insert(BB);
                if (lastRemoved) {
                    // Last segment is removed
                    if (checkRegionExit(i)) {
                        // Insert right bracket
                        program += "}\n";
                    }
                }
                lastRemoved = false;
                copyEn = true;
            }

            if (copyEn)
                program += translateSource(sourceBuf, lineStart, colStart);
            sourceBuf.clear();
            if (done) break;
        }
        program += insertInputExit(BBset, &(F.getEntryBlock()), endBB, endBB, 0);

        program += "}\n";

        std::string prologue = generateInputMain(btID);
        program = prologue + program;

        outFile << program;

        generateInputHeader(192, btID);

    }

    void generateInputHeader(int REGWIDTH, int numExit) {
        std::ofstream outFile("input.h");
        std::string program;
        program = generateStructHeader(REGWIDTH);
        outFile << program;
        outFile << numExit << "\n";
    }

    int getInputLengthBB(BasicBlock &BB, int start) {
        int index = start;
        int totalLength = start;
        bool hasInputHeader = false;
        for (auto inst_it = BB.begin(); inst_it != BB.end(); inst_it++) {
            Instruction *inst = &*inst_it;
            if (isa<CallInst>(*inst)) {
                auto *tmp = dyn_cast<llvm::CallInst>(&*inst);
                Function *foo = tmp->getCalledFunction();
                if (foo->getName().contains("Input_header")) {
                    // record debug info
                    if (DILocation *Loc = inst->getDebugLoc()) {
                        unsigned line = Loc->getLine();
                        unsigned col = Loc->getColumn();
                        inputInstLoc[line] = std::make_pair(col, &*inst);
                    }
                    hasInputHeader = true;

                    Value* arg = tmp->getArgOperand(1);
                    auto* argInt = dyn_cast<ConstantInt>(arg);
                    int length = argInt->getSExtValue();
                    // inst->print(errs());
                    // errs() << "\n index: " << index << " length: " << length;
                    if (index + length > IO_W) {
                        inputInstRange[&*inst] = std::make_tuple(index, IO_W-1, index+length-IO_W-1);
                        index = index+length-IO_W;
                    } else {
                        inputInstRange[&*inst] = std::make_tuple(index, index+length-1, -1);
                        index = index+length;
                    }
                    totalLength = (totalLength+length >= IO_W) ? totalLength+length-IO_W : totalLength+length;
                    // errs() << ", totalLength: " << totalLength << "\n";
                }
            }
        }
        if (hasInputHeader) {
            return totalLength;
        }
        else {
            return -1;
        }
    }

    BasicBlock* inferInputFunctions(Function &F, BasicBlock* &startBB, BasicBlock* &endBB) {
        std::vector<std::pair<BasicBlock*, int>> workstack;
        std::set<Value*> visited; // visited and contains Input_head

        BasicBlock *bb = &F.getEntryBlock();
        startBB = bb;
        int index = getInputLengthBB(*bb, 0);
        workstack.emplace_back(bb, index);
        if (index != -1) {
            visited.insert(bb);
        }

        if (!(endBB = getBBwithFunc(F, "Input_done", inputDoneLine))) {
            errs() << "Input_done not found!\n";
            exit(1);
        }

        while (!workstack.empty()) {
            auto bb_pair = workstack.back();
            workstack.pop_back();
            
            bb = bb_pair.first;
            index = bb_pair.second;
            for (auto succ = succ_begin(bb), succEnd = succ_end(bb); succ != succEnd; ++succ) {
                BasicBlock* bb_succ = *succ;
                int newIndex = getInputLengthBB(*bb_succ, index);
                if (newIndex != -1) {
                    if (visited.find(&*bb_succ) != visited.end()) {
                        printBasicBlock(*bb_succ);
                        errs() << "Error: Input_header already visited in this basic block\n";
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

        return endBB;
    }

    protected:
    
    virtual BitVector* getLiveoutVariables(std::set<BasicBlock*> &BBset, BasicBlock &endBB) = 0;

    virtual std::string insertInputExit(std::set<BasicBlock*> &BBset, BasicBlock* startBB, BasicBlock* endBB, BasicBlock* doneBB, int btID) = 0;
    
};
}

#endif

