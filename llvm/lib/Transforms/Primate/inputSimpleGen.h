//	File "inputSimpleGen.h"
//	
//  Author: Rui Ma
////////////////////////////////////////////////////////////////////////////////
#ifndef __INPUTSIMPLEGEN_H__
#define __INPUTSIMPLEGEN_H__

#include "PrimateBFUGenBase.h"

#define IO_W 32
#define RATE (0.01)

using namespace llvm;

namespace {

class inputSimpleGen : virtual public PrimateBFUGenBase{
public:
    // domain vector to store all definitions and function arguments
    int inputDoneLine;

    // tuple: start index; end index; end index of the second flit if the header spans across two flits, default -1
    ValueMap<Value*, std::tuple<int, int, int>> inputInstRange;
    // TODO: line, col, Inst_ptr, bfc_name, bfu_name; assume each line has only one Input_header
    std::map<int, std::tuple<int, Value*, std::string, std::string>> blueInstLoc;
    ValueMap<Value*, std::string> bfc2bfu;

    // bfu_name, wb
    std::map<std::string, bool> bfu_list;
    // map<bfc_name, tuple<bfu_name, opcode, imm>>
    std::map<std::string, std::tuple<std::string, int, int>> bfc_list;
    std::set<std::string> bfu_covered;
    std::set<std::string> bfu_not_fully_covered;

    struct varMeta_t {
        std::string name;
        bool isInt;

        varMeta_t(std::string name, bool isInt = false) : name(name), isInt(isInt) {}
    };
    
    inputSimpleGen() {
        bfu_list["lock"] = false;
        bfc_list["primate_lock"] = std::make_tuple("lock", 0, 0);
        bfc_list["primate_unlock"] = std::make_tuple("lock", 1, 0);

        bfu_list["flow_table_read"] = true;
        bfc_list["flow_table_read"] = std::make_tuple("flow_table_read", 0, 0);

        bfu_list["flow_table_write"] = false;
        bfc_list["flow_table_insert"] = std::make_tuple("flow_table_write", 1, 0);
        bfc_list["flow_table_update"] = std::make_tuple("flow_table_write", 2, 0);
        bfc_list["flow_table_delete"] = std::make_tuple("flow_table_write", 3, 0);
        
        bfu_list["dynamicMem"] = true;
        bfc_list["dymem_new"] = std::make_tuple("dynamicMem", 0, 0);
        bfc_list["dymem_lookup"] = std::make_tuple("dynamicMem", 1, 0);
        bfc_list["dymem_update"] = std::make_tuple("dynamicMem", 2, 0);
        bfc_list["dymem_free"] = std::make_tuple("dynamicMem", 3, 0);
    }

    ~inputSimpleGen(){}

    std::string generateInputMain(int numExit) {
        std::string program =   "#include \"inputUnit.h\"\n"
                                "void inputUnit::inputUnit_main() {\n"
                                "    primate_ctrl_iu::cmd_t cmd;\n\n"
                                "    stream_in.reset();\n"
                                "    stream_out.reset();\n"
                                "    cmd_in.reset();\n"
                                "    bfu_out.reset();\n";

        for (auto name = bfu_covered.begin(); name != bfu_covered.end(); name++) {
            program += ("    " + (*name) + "_req.reset();\n");
            program += ("    " + (*name) + "_rsp.reset();\n");
        }

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
                    "            bfu_out.write_last(tag, 0);\n";
        
        int i = 1;
        for (auto name = bfu_not_fully_covered.begin(); name != bfu_not_fully_covered.end(); name++) {
            program += ("        } else if (opcode & 0x30 == " + std::to_string(i << 4) + ") {\n");
            program += ("            " + (*name) + "_req.write(bfu_in_pl_t(tag, opcode, cmd.ar_imm, cmd.ar_bits));\n");
            program += ("            bfu_out_pl_t tmp = " + (*name) + "_rsp.read();\n");
            if (bfu_list[*name]) {
                program += ("            bfu_out.write_last(tag, 0);\n");
            } else {
                program += ("            bfu_out.write(tag, tmp.flag, cmd.ar_rd, tmp.bits, true);\n");
            }
            if (i > 3) {
                errs() << "too many bfus not fully covered\n";
                exit(1);
            }
            i++;
        }

        program +=  "        } else {\n"
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

    std::string getVarMetaName(Value* op, std::string name) {
        std::string res = name;
        if (isa<AllocaInst>(*op)) {
            auto *inst = dyn_cast<AllocaInst>(op);
            res = (varNameMap[&*inst] + "." + res);
        } else if (isa<GetElementPtrInst>(*op)) {
            auto *inst = dyn_cast<GetElementPtrInst>(op);
            Type *baseType = inst->getSourceElementType();
            Value* baseVar = inst->getOperand(0);
            unsigned idx = (dyn_cast<ConstantInt>(inst->getOperand(2)))->getZExtValue();
            std::string typeName = (baseType->getStructName()).str();
            typeName.erase(0, 7);
            std::string fieldName = std::get<0>((*varTypeMap[typeName])[idx]);
            // errs() << idx << " " << typeName << " " << fieldName << "\n";
            res = (fieldName + "." + res);
            res = getVarMetaName(baseVar, res);
        }
        return res;
    }

    varMeta_t* getVarMeta(Value* op) {
        if (isa<AllocaInst>(*op)) {
            auto *inst = dyn_cast<AllocaInst>(op);
            Type *varType = inst->getAllocatedType();
            std::string name = varNameMap[&*inst];
            bool isInt = false;
            if (varType->isIntegerTy()) {
                isInt = true;
            }
            varMeta_t *res = new varMeta_t(name, isInt);
            return res;
        } else if (isa<GetElementPtrInst>(*op)) {
            auto *inst = dyn_cast<GetElementPtrInst>(op);
            Type *varType = inst->getResultElementType();
            bool isInt = false;
            if (varType->isIntegerTy()) {
                isInt = true;
            }
            std::string name = getVarMetaName(op, "");
            name.pop_back();
            varMeta_t *res = new varMeta_t(name, isInt);
            return res;
        }
        return NULL;
    }

    varMeta_t* getBFCInputMeta(Instruction* inst) {
        if (isa<CallInst>(*inst)) {
            auto *call = dyn_cast<llvm::CallInst>(&*inst);
            if (call->arg_size() > 0) {
                Value* op = call->getArgOperand(0);
                return getVarMeta(op);
            }
        }
        return NULL;
    }

    varMeta_t* getBFCOutputMeta(Instruction* inst) {
        if (isa<CallInst>(*inst)) {
            auto *call = dyn_cast<llvm::CallInst>(&*inst);
            if (call->arg_size() > 1) {
                Value* op = call->getArgOperand(1);
                return getVarMeta(op);
            }
        }
        return NULL;
    }

    std::string translateSource(std::string source, int lineStart, int colStart) {
        std::string program;
        int line = lineStart;
        int col = colStart;
        std::istringstream iss(source);
        std::string lineBuf;
        // errs() << "start translate:\n" << source << "\n";
        while (std::getline(iss, lineBuf)) {
            auto blueInstMeta = blueInstLoc.find(line);
            if (blueInstMeta != blueInstLoc.end()) {
                std::string bfc_name = std::get<2>(blueInstMeta->second);
                Instruction *inst = dyn_cast<Instruction>(std::get<1>(blueInstMeta->second));
                // inst->print(errs());
                // errs() << "\n";
                if (bfc_name.find("Input_simple") != std::string::npos) {
                    // find input call location and get variable name
                    std::size_t index0 = lineBuf.find("top_intf.Input_simple");
                    std::size_t index1 = lineBuf.find("(", index0);
                    std::size_t index2 = lineBuf.find(")", index0);
                    std::size_t index3 = lineBuf.find(";", index0);
                    std::string varName = lineBuf.substr(index1+1, index2-index1-1);
                    varName.erase(std::remove_if(varName.begin(), varName.end(), ::isspace), varName.end());
                    // construct substitution
                    std::string newCode;
                    newCode += "payload = stream_in.read();\n";
                    newCode += (varName + ".set(payload.data);\n");

                    // substitute the input call
                    lineBuf.replace(index0, index3-index0+1, newCode);
                } else if (bfc_name.find("Output_simple") != std::string::npos) {
                    std::size_t index0 = lineBuf.find("top_intf.Output_simple");
                    std::size_t index1 = lineBuf.find("(", index0);
                    std::size_t index2 = lineBuf.find(")", index0);
                    std::size_t index3 = lineBuf.find(";", index0);
                    std::string varName = lineBuf.substr(index1+1, index2-index1-1);
                    varName.erase(std::remove_if(varName.begin(), varName.end(), ::isspace), varName.end());
                    // construct substitution
                    std::string newCode;
                    newCode += ("stream_out.write(primate_io_payload_t(" + varName + ".to_uint(), tag, 0, true));\n");

                    // substitute the output call
                    lineBuf.replace(index0, index3-index0+1, newCode);
                } else {
                    std::string bfu_name = std::get<3>(blueInstMeta->second);
                    std::size_t index0 = lineBuf.find(bfc_name);
                    std::size_t index3 = lineBuf.find(";", index0);
                    varMeta_t *varInMeta = getBFCInputMeta(inst);
                    varMeta_t *varOutMeta = getBFCOutputMeta(inst);
                    int opcode = std::get<1>(bfc_list[bfc_name]);
                    int imm = std::get<2>(bfc_list[bfc_name]);

                    std::string newCode;
                    if (varInMeta != NULL) {
                        newCode += (bfu_name + "_req.write(bfu_in_pl_t(tag, " + std::to_string(opcode) +
                            ", " + std::to_string(imm) + ", " + varInMeta->name);
                        if (!(varInMeta->isInt)) newCode += ".to_uint()";
                    } else {
                        newCode += (bfu_name + "_req.write(bfu_in_pl_t(tag, " + std::to_string(opcode) +
                            ", " + std::to_string(imm) + ", 0");
                    }
                    newCode += "));\n";

                    if (varOutMeta != NULL) {
                        newCode += ("bfu_out_pl_t tmp = " + bfu_name + "_rsp.read();\n");
                        if (varOutMeta->isInt) {
                            newCode += (varOutMeta->name + " = tmp.bits;\n");
                        } else {
                            newCode += (varOutMeta->name + ".set(tmp.bits);\n");
                        }
                    } else {
                        newCode += (bfu_name + "_rsp.read();\n");
                    }
                    lineBuf.replace(index0, index3-index0+1, newCode);
                }
            } else if ((lineBuf.find("return;") != std::string::npos) || (lineBuf.find("return ") != std::string::npos)) {
                std::size_t index0 = lineBuf.find("return");
                std::size_t index1 = lineBuf.find(";", index0);
                std::string newCode;

                newCode = "bfu_out.write_last(tag, bt0);\nreturn;\n";
                lineBuf.replace(index0, index1-index0+1, newCode);
            }
            line++;
            program += lineBuf;
            if (!iss.eof()) {
                program += "\n";
            }
        }

        return program;
    }

    void extractInputFunction(Function &F) {
        std::ifstream srcFile(filename);
        std::ofstream outFile("inputSpec.cpp");
        // double threshold = 0.5;
        // double nt = threshold * avgExeCountPerInst;
        identifyBFCs(F);

        double threshold = exploreFunction(srcFile, 0.5, 0, 0);
        double nt = threshold * avgExeCountPerInst;
        srcFile.seekg(0);

        std::set<BasicBlock*> BBset;
        std::string program;

        program +=  "void inputUnit::inputUnit_core() {\n"
                    "    primate_stream_512_4::payload_t payload;\n";

        int line = 1;
        int col = 1;
        int lineClose = 1;
        int colClose = 1;
        bool closeValid = false;
        int btID = 1;
        bool lastRemoved = false;
        std::string sourceBuf;
        getSource(srcFile, sourceBuf, line, col, profSegments[0][0], profSegments[0][1]+1, "Input_done");
        col++; // skip the first "{"
        sourceBuf.clear();
        for (int i = 0; i < numProfSegments; i++) {
            if (profSegments[i][3] == 0) continue;
            bool copyEn;

            int lineStart = line;
            int colStart = col;
            getSource(srcFile, sourceBuf, line, col, profSegments[i+1][0], profSegments[i+1][1], "Input_done");
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
                        if (!getRegionCloseLoc(profSegments[i][0], profSegments[i][1], lineClose, colClose)) {
                            errs() << "unable to find the closing location of the region\n";
                        }
                        // errs() << "lineClose: " << lineClose << ", colClose: " << colClose << "\n";
                        closeValid = true;
                        // Insert early exit statements
                        // outFile << "cout << \"early exit\\n\";\n";
                        BasicBlock *nextBB = BB;
                        int j = 1;
                        while (nextBB == NULL) {
                            nextBB = getBBfromLoc(profSegments[i+j][0], profSegments[i+j][1], profSegments[i+j+1][0], profSegments[i+j+1][1]);
                            j++;
                        }
                        BasicBlock *prevBB = NULL;
                        j = 1;
                        while (prevBB == NULL) {
                            prevBB = getBBfromLoc(profSegments[i-j][0], profSegments[i-j][1], profSegments[i-j+1][0], profSegments[i-j+1][1]);
                            j++;
                        }
                        program += insertInputExit(BBset, &(F.getEntryBlock()), prevBB, nextBB, btID);
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
                        program += "}\n";
                    }
                }
                lastRemoved = false;
                copyEn = true;
            }

            if (copyEn)
                program += translateSource(sourceBuf, lineStart, colStart);

            sourceBuf.clear();
        }

        identifyCoveredBFCs(F, BBset);

        std::string prologue = generateInputMain(btID);
        program = prologue + program;

        outFile << program;

        generateInputHeader(272, btID);

    }

    void generateInputHeader(int REGWIDTH, int numExit) {
        std::ofstream outFile("input.h");
        std::string program;
        program = generateStructHeader(REGWIDTH);
        outFile << program;
        outFile << numExit << "\n";
    }

    void identifyBFCs(Function &F) {
        for (BasicBlock &BB : F) {
            for (auto inst_it = BB.begin(); inst_it != BB.end(); inst_it++) {
                Instruction *inst = &*inst_it;
                if (isa<CallInst>(*inst)) {
                    auto *tmp = dyn_cast<llvm::CallInst>(&*inst);
                    Function *foo = tmp->getCalledFunction();
                    auto name = foo->getName();
                    if (DILocation *Loc = inst->getDebugLoc()) {
                        unsigned line = Loc->getLine();
                        unsigned col = Loc->getColumn();
                        if (name.contains("Input_simple")) {
                            blueInstLoc[line] = std::make_tuple(col, &*inst, "Input_simple", "Input");
                        } else if (name.contains("Output_simple")) {
                            blueInstLoc[line] = std::make_tuple(col, &*inst, "Output_simple", "Output");
                        } else {
                            for (auto bfc_it = bfc_list.begin(); bfc_it != bfc_list.end(); bfc_it++) {
                                std::string bfc_name = bfc_it->first;
                                std::string bfu_name = std::get<0>(bfc_it->second);
                                if (name.contains(bfc_name)) {
                                    blueInstLoc[line] = std::make_tuple(col, &*inst, bfc_name, bfu_name);
                                    if (bfc2bfu.find(&*inst) == bfc2bfu.end()) {
                                        bfc2bfu[&*inst] = bfu_name;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void identifyCoveredBFCs(Function &F, std::set<BasicBlock*> BBset) {
        std::set<Value*> bfus;
        for (BasicBlock &BB : F) {
            if (BBset.find(&BB) != BBset.end()) {
                for (Instruction &I : BB) {
                    if (bfc2bfu.find(&I) != bfc2bfu.end()) {
                        bfu_covered.insert(bfc2bfu[&I]);
                        bfus.insert(&I);
                    }
                }
            }
        }
        for (BasicBlock &BB : F) {
            if (BBset.find(&BB) == BBset.end()) {
                for (Instruction &I : BB) {
                    if (bfus.find(&I) != bfus.end()) {
                        bfu_not_fully_covered.insert(bfc2bfu[&I]);
                    }
                }
            }
        }
    }

    BitVector* getLiveoutVariables(std::set<BasicBlock*> &BBset, BasicBlock &nextBB) {
        BitVector* updated = new BitVector(domainSize, false);
        for (auto bb_it = BBset.begin(); bb_it != BBset.end(); bb_it++) {
            for (auto inst_it = (*bb_it)->begin(); inst_it != (*bb_it)->end(); inst_it++) {
                Instruction *tempInst = &*inst_it;
                // tempInst->print(errs());
                // errs() << "\n";
                if (isa<CallInst>(*tempInst)) {
                    // input header
                    auto *inst = dyn_cast<llvm::CallInst>(&*tempInst);
                    Function* foo = inst->getCalledFunction();
                    if (foo->getName().contains("Input_simple")) {
                        Value* ptrOp = inst->getOperand(1);
                        (*updated)[(*valueToBitVectorIndex)[(*aliasMap)[ptrOp]]] = true;
                    } else if (!(foo->getName().contains("Output_simple"))) {
                        // BFUs
                        if (inst->arg_size() > 1) {
                            Value* ptrOp = inst->getOperand(1);
                            (*updated)[(*valueToBitVectorIndex)[(*aliasMap)[ptrOp]]] = true;
                        }
                    }
                } else if (isa<StoreInst>(*tempInst)) {
                    // store the struct
                    auto *inst = dyn_cast<llvm::StoreInst>(&*tempInst);
                    Value *ptrOp = inst->getPointerOperand();
                    (*updated)[(*valueToBitVectorIndex)[(*aliasMap)[ptrOp]]] = true;
                } else if (!isa<AllocaInst>(*tempInst)) {
                    (*updated)[(*valueToBitVectorIndex)[tempInst]] = true;
                }
            }
        }

        (*updated) &= (*(*in)[&nextBB]);

        return updated;
    }

    std::string insertInputExit(std::set<BasicBlock*> &BBset, BasicBlock* startBB, BasicBlock* endBB, BasicBlock* nextBB, int btID) {
        std::set<BasicBlock*> intersect;
        std::set<BasicBlock*> reachable;
        std::vector<BasicBlock*> workstack;
        std::vector<BasicBlock*> path;
        std::set<BasicBlock*> visited;

        reachable.insert(endBB);
        workstack.emplace_back(startBB);
        while (!workstack.empty()) {
            BasicBlock *BB = workstack.back();
            workstack.pop_back();
            visited.insert(BB);

            path.emplace_back(BB);

            for (auto succ = succ_begin(BB), succEnd = succ_end(BB); succ != succEnd; ++succ) {
                BasicBlock* bb_succ = *succ;
                if (reachable.find(bb_succ) != reachable.end()) {
                    for (auto it = path.begin(); it != path.end(); it++) {
                        reachable.insert(*it);
                    }
                    path.clear();
                    continue;
                } else if (visited.find(bb_succ) == visited.end()) {
                    workstack.emplace_back(bb_succ);
                }
            }
        }

        if (!path.empty()) {
            for (auto it = path.begin(); it != path.end(); it++) {
                reachable.insert(*it);
            }
        }
        
        for (auto bb_it = reachable.begin(); bb_it != reachable.end(); bb_it++) {
            if (BBset.find(*bb_it) != BBset.end()) {
                intersect.insert(*bb_it);
            }
        }

        BitVector* liveout = getLiveoutVariables(intersect, *nextBB);

        int slot = 0;
        std::string exitCode;
        for (int i=0; i < liveout->size(); i++) {
            if ( (*liveout)[i] ) {
                Value *var = (*bvIndexToInstrArg)[i];
                bool pack = false;
                if (isa<AllocaInst>(*var)) {
                    auto *inst = dyn_cast<AllocaInst>(var);
                    Type *varType = inst->getAllocatedType();
                    if (isa<IntegerType>(*varType)) {
                        pack = false;
                    } else if (isa<StructType>(*varType)) {
                        pack = true;
                    }
                }
                if (varNameMap.find(var) != varNameMap.end()) {
                    std::string name = varNameMap[var];
                    int reg = varRegMap[var];
                    if (name == "top_intf") {
                        continue;
                    } else if (pack) {
                        name += ".to_uint()";
                    }
                    if (slot == 0) {
                        exitCode += ("bfu_out.write(tag, bt" + std::to_string(btID) + ", " + std::to_string(reg) + ", " + name + ", ");
                        slot = 1;
                    } else {
                        exitCode += std::to_string(reg) + ", " + name + ");\n";
                        slot = 0;
                    }
                }
            }
        }
        if (slot == 1) {
            exitCode += "true);\n";
        } else {
            exitCode.erase(exitCode.length()-3);
            exitCode += ", true);\n";
        }

        exitCode += "return;\n";
        
        return exitCode;
    }

};
}

#endif

