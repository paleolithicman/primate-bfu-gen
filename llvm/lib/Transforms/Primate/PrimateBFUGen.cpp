//  PrimateBFUGen.cpp
//  To generate architectural parameters for Primate template
//
//  Based on code from Todd C. Mowry
//  Modified by Arthur Peters
//  Modified by Ankit Goyal
//  Modified by Rui Ma
/////////////////////////////////////////////////////////////////////////////////////

/******************************************************************************

Liveness: OUT[n] = UNION_{s E succ[n]} IN[s]  //meet
IN[n] = GEN[n] U (OUT[n] - KILL[n]) //transfer function

Flow Direction: Backward
A BitVector stored at each node for IN and OUT. Bit vector contains an entry for all the values

Boundary Conditions: empty set for flow value. identified by no successors.

 *********************************************************************************/

#include<llvm/Pass.h>
#include<llvm/IR/DebugInfo.h>
#include<llvm/IR/Function.h>
#include<llvm/IR/Module.h>
#include<llvm/IR/Metadata.h>
#include<llvm/Support/raw_ostream.h>
#include<llvm/Support/FormattedStream.h>
#include<llvm/IR/InstIterator.h>
#include<llvm/IR/Instruction.h>
#include<llvm/IR/AssemblyAnnotationWriter.h>
#include<llvm/ADT/BitVector.h>
#include<llvm/IR/ValueMap.h>
#include<llvm/ADT/DenseMap.h>

#include<llvm/IR/LegacyPassManager.h>
#include<llvm/Transforms/IPO/PassManagerBuilder.h>

#include "rapidjson/document.h"
#include "dataflow.h"

#include <ostream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <algorithm>
#include <random>
#include <math.h>
#include <map>
#include <tuple>
#include <string>

#define MAX_BR_LEVEL 2
#define MAX_PERF 0
#define BALANCE 1
#define IO_W 64
#define RATE (0.05)

using namespace llvm;

namespace {

    class PrimateBFUGen : public ModulePass, public DataFlow<BitVector>, public AssemblyAnnotationWriter{

        public:
            static char ID;

            // set forward false in the constructor DataFlow()
            PrimateBFUGen() : DataFlow<BitVector>(false), ModulePass(ID) {
                bvIndexToInstrArg = new std::vector<Value*>();
                valueToBitVectorIndex = new ValueMap<Value*, int>();
                instrInSet = new ValueMap<const Instruction*, BitVector*>();
                aliasMap = new ValueMap<Value*, Value*>();
                branchLevel = new ValueMap<Value*, int>();
            }

            struct ptrInfo_t {
                Value *base;
                int offset;
                ptrInfo_t(Value *base = NULL, int offset = 0) : base(base), offset(offset) {}
            };

            // domain vector to store all definitions and function arguments
            std::string filename;
            unsigned long long numTotalInst;
            unsigned long long numTotalExeCount;
            int numProfSegments;
            int **profSegments;
            int numProfRegions;
            int **profRegions;
            int numProfBranches;
            int **profBranches;
            int *numInstPerSeg;
            double avgExeCountPerInst;
            int numInst;
            int numLine;
            int inputDoneLine;
            std::map<int, std::map<int, Value*>*> lineToBBs;
            ValueMap<Value*, std::string> varNameMap;
            ValueMap<Value*, int> varRegMap;
            std::map<std::string, std::vector<std::pair<std::string, int>>*> varTypeMap;

            // tuple: start index; end index; end index of the second flit if the header spans across two flits, default -1
            ValueMap<Value*, std::tuple<int, int, int>> inputInstRange;
            // TODO: line, col, Inst_ptr; assume each line has only one Input_header
            std::map<int, std::pair<int, Value*>> inputInstLoc;

            std::vector<Value*> domain;
            std::vector<Value*> *bvIndexToInstrArg;                 //map values to their bitvector
            ValueMap<Value*, int> *valueToBitVectorIndex;           //map values (args and variables) to their bit vector index
            ValueMap<const Instruction*, BitVector*> *instrInSet;     //IN set for an instruction inside basic block
            ValueMap<Value*, Value*> *aliasMap;
            ValueMap<Value*, int> *branchLevel;
            std::vector<unsigned> *gatherModes;
            std::map<unsigned, std::vector<unsigned>*> *fieldIndex;
            ValueMap<Value*, ptrInfo_t*> *pointerMap;
            ValueMap<Value*, std::map<Value*, bool>*> dependencyForest;
            ValueMap<Value*, std::map<Value*, bool>*> dependencyForestOp;
            ValueMap<Value*, int> instPriority;
            std::set<Value*> loadMergedInst;
            std::set<Value*> unmergeableLoad;
            std::set<Value*> unmergeableStore;
            std::set<Value*> combinedBranchInst;
            std::map<BasicBlock*, std::set<Value*>*> frontiers;
            std::map<BasicBlock*, double> bbWeight;
            std::map<BasicBlock*, int> bbNumInst;
            std::map<BasicBlock*, int> bbNumVLIWInst;

            int domainSize;
            int numArgs;
            int numInstr;
            int numALU_min;
            
            int live[50];
            unsigned int n = 0;

            std::ofstream primateCFG;
            std::ofstream interconnectCFG;
            std::ofstream primateHeader;
            std::ofstream assemblerHeader;

            //print functions
            //live variables before each basic block
            virtual void emitBasicBlockStartAnnot(const BasicBlock *bb, formatted_raw_ostream &os) {
                os << "; ";
                if (!isa<PHINode>(*(bb))) {
                    const BitVector *bv = (*in)[&*bb];

                    for (int i=0; i < bv->size(); i++) {
                        if ( (*bv)[i] ) {
                            os << (*bvIndexToInstrArg)[i]->getName();
                            os << ", ";
                        }
                    }
                }
                os << "\n";
            }

            //live variables after each basic block
            virtual void emitBasicBlockEndAnnot(const BasicBlock *bb) {
                errs() << "; ";
                if (!isa<PHINode>(*(bb))) {
                    const BitVector *bv = (*out)[&*bb];

                    for (int i=0; i < bv->size(); i++) {
                        if ( (*bv)[i] ) {
                            errs() << (*bvIndexToInstrArg)[i]->getName();
                            errs() << ", ";
                        }
                    }
                }
                errs() << "\n";
            }

            //live variables before each instruction: used for computing histogram
            virtual void emitInstructionAnnot(const Instruction *i, formatted_raw_ostream &os) {
                os << "; ";
                if (!isa<PHINode>(*(i))) {
                    const BitVector *bv = (*instrInSet)[&*i];
/*                    
                    live[bv->size()]++;
                    if(bv->size() > n)
                        n = bv->size();
*/                    
                    for (int i=0; i < bv->size(); i++) {
                        if ( (*bv)[i] ) {
                            os << (*bvIndexToInstrArg)[i]->getName();
                            os << ", ";
                        }
                    }
                }
                os << "\n";
            }

            //implementation of functions

            //set the boundary condition for block
            //explicit constructor of BitVector
            virtual void setBoundaryCondition(BitVector *blockBoundary) {
                *blockBoundary = BitVector(domainSize, false); 
            }

            //union (bitwise OR) operator '|=' overriden in BitVector class
            virtual void meetOp(BitVector* lhs, const BitVector* rhs){
                *lhs |= *rhs; 
            }

            //empty set initially; each bit represent a value
            virtual BitVector* initializeFlowValue(BasicBlock& b, SetType setType){ 
                return new BitVector(domainSize, false); 
            }


            //transfer function:
            //IN[n] = USE[n] U (OUT[n] - DEF[n])
            
            virtual BitVector* transferFn(BasicBlock& bb) {
                BitVector* outNowIn = new BitVector(*((*out)[&bb]));

                BitVector* immIn = outNowIn; // for empty blocks
                Instruction* tempInst;
                bool breakme=false;
                // go through instructions in reverse
                BasicBlock::iterator ii = --(bb.end()), ib = bb.begin();
                while (true) {

                    // inherit data from next instruction
                    tempInst = &*ii;
                    immIn = (*instrInSet)[tempInst];            
                    *immIn = *outNowIn;

                    // if this instruction is a new definition, remove it
                    if (isDefinition(tempInst)){
                        (*immIn)[(*valueToBitVectorIndex)[tempInst]] = false;
                    }

                    if (isa<CallInst>(*tempInst)) {
                        auto *inst = dyn_cast<llvm::CallInst>(&*tempInst);
                        Function* foo = inst->getCalledFunction();
                        if (foo->getName().contains("lifetime.start")) {
                            Value* ptrOp = inst->getOperand(1);
                            (*immIn)[(*valueToBitVectorIndex)[(*aliasMap)[ptrOp]]] = false;
                            // tempInst->print(errs());
                            // errs() << " kill " << (*aliasMap)[ptrOp]->getName() << "\n";
                        }
                    }

                    // add the arguments, unless it is a phi node
                    if (!isa<PHINode>(*ii)) {
                        // skip if can be combined into a single branch instruction
                        if (combinedBranchInst.find(&*ii) == combinedBranchInst.end()) {
                            // skip getElementPtrInst, bitCast, lifetime call instructions
                            if (!(isa<GetElementPtrInst>(*ii) || isa<BitCastInst>(*ii) || isLifetimeCall(&*ii))) {
                                User::op_iterator OI, OE;
                                for (OI = tempInst->op_begin(), OE=tempInst->op_end(); OI != OE; ++OI) {
                                    if (isa<Instruction>(*OI) || isa<Argument>(*OI)) {
                                        Value *op = (*aliasMap)[*OI];
                                        (*immIn)[(*valueToBitVectorIndex)[op]] = true;
                                    }
                                }
                            }
                        }
                    } else if(isa<PHINode>(*ii)) {
                        PHINode* phiNode = cast<PHINode>(&*ii);
                        for (int incomingIdx = 0; incomingIdx < phiNode->getNumIncomingValues(); incomingIdx++) {
                            Value* val = phiNode->getIncomingValue(incomingIdx);
                            if (isa<Instruction>(val) || isa<Argument>(val)) {
                                int valIdx = (*valueToBitVectorIndex)[(*aliasMap)[val]];
                                BasicBlock* incomingBlock = phiNode->getIncomingBlock(incomingIdx);
                                if ((*neighbourSpecificValues).find(incomingBlock) == (*neighbourSpecificValues).end())
                                    (*neighbourSpecificValues)[incomingBlock] = new BitVector(domainSize);
                                (*(*neighbourSpecificValues)[incomingBlock]).set(valIdx);                                
                            }
                        }
                    }

                    outNowIn = immIn;

                    if (ii == ib) break;

                    --ii;
                }

                return immIn;
            }

            void printBasicBlock(BasicBlock &B) {
                for (auto inst_it = B.begin(); inst_it != B.end(); inst_it++) {
                    inst_it->print(errs());
                    errs() << "\n";
                }
            }

            bool isLifetimeCall(Instruction *ii) {
                bool res = false;
                if (isa<CallInst>(*ii)) {
                    auto *inst = dyn_cast<llvm::CallInst>(&*ii);
                    Function* foo = inst->getCalledFunction();
                    if (foo->getName().contains("lifetime")) {
                        res = true;
                    }
                }
                return res;
            }

            bool isDefinition(Instruction *ii) {
                return !(ii->isTerminator()) ;
            }

            void calculate(const Instruction *i) {
                //static unsigned int live[50], n=0;
                int count=0;
                
                if (!isa<PHINode>(*(i))) {
                    const BitVector *bv = (*instrInSet)[&*i];
                    
                    for(int i=0;i<bv->size();i++){
                        if((*bv)[i])
                            count++;
                    }
                    
                    if(count > n) {
                        n = count+1;
                    }
                    
                    live[count]++;
                }
            }

            bool isBlueCall(Instruction *ii) {
                bool res = false;
                if (isa<llvm::CallInst>(&*ii)) {
                    MDNode *PrimateMetadata = ii->getMetadata("primate");
                    if (PrimateMetadata != NULL) {
                        if (cast<MDString>(PrimateMetadata->getOperand(0))->getString() == "blue") { // this is probably not safe to assume the type, you probably want to type check...
                            res = true;
                        }
                    }
                }
                return res;
            }

            unsigned getArrayWidth(ArrayType &a, unsigned start) {
                unsigned num_elem = a.getNumElements();
                auto elem = a.getElementType();
                unsigned elemWidth;
                if (isa<llvm::IntegerType>(*elem)) {
                    elemWidth = elem->getIntegerBitWidth();
                } else if (isa<llvm::ArrayType>(*elem)) {
                    auto *selem = dyn_cast<llvm::ArrayType>(elem);
                    elemWidth = getArrayWidth(*selem, 0);
                } else if (isa<llvm::StructType>(*elem)) {
                    auto *selem = dyn_cast<llvm::StructType>(elem);
                    elemWidth = getStructWidth(*selem, 0, false);
                }
                return (start + num_elem * elemWidth);
            }

            unsigned getArrayWidthArcGen(ArrayType &a, unsigned start) {
                unsigned width = start;
                auto elem = a.getElementType();
                unsigned elemWidth = 0;
                if (isa<llvm::IntegerType>(*elem)) {
                    elemWidth = elem->getIntegerBitWidth();
                    // insert new gather mode if doesn't exist
                    if (std::find(gatherModes->begin(), gatherModes->end(), elemWidth) == gatherModes->end()) {
                        gatherModes->push_back(elemWidth);
                    }
                }
                for (int i = 0; i < a.getNumElements(); i++) {
                    if (fieldIndex->find(width) == fieldIndex->end()) {
                        (*fieldIndex)[width] = new std::vector<unsigned>();
                    }
                    if (isa<llvm::IntegerType>(*elem)) {
                        // the width of all numbers possibly stored at each index
                        if (std::find((*fieldIndex)[width]->begin(), (*fieldIndex)[width]->end(), elemWidth) == (*fieldIndex)[width]->end()) {
                            (*fieldIndex)[width]->push_back(elemWidth);
                        }
                        width += elemWidth;
                    } else if (isa<llvm::ArrayType>(*elem)) {
                        auto *selem = dyn_cast<llvm::ArrayType>(elem);
                        unsigned elemWidth = getArrayWidthArcGen((*selem), width);
                        width = elemWidth;
                    } else if (isa<llvm::StructType>(*elem)) {
                        auto *selem = dyn_cast<llvm::StructType>(elem);
                        unsigned elemWidth = getStructWidth((*selem), width, true);
                        width = elemWidth;
                    }
                }
                return width;
            }

            unsigned getStructWidth(StructType &s, unsigned start, const bool arcGen) {
                unsigned width = start;
                for (auto elem = s.element_begin(); elem != s.element_end(); elem++) {
                    if (arcGen) {
                        if (fieldIndex->find(width) == fieldIndex->end()) {
                            (*fieldIndex)[width] = new std::vector<unsigned>();
                        }
                    }
                    if (isa<llvm::IntegerType>(**elem)) {
                        unsigned elemWidth = (*elem)->getIntegerBitWidth();
                        if (arcGen) {
                            // insert new gather mode if doesn't exist
                            if (std::find(gatherModes->begin(), gatherModes->end(), elemWidth) == gatherModes->end()) {
                                gatherModes->push_back(elemWidth);
                            }
                            // the width of all numbers possibly stored at each index
                            if (std::find((*fieldIndex)[width]->begin(), (*fieldIndex)[width]->end(), elemWidth) == (*fieldIndex)[width]->end()) {
                                (*fieldIndex)[width]->push_back(elemWidth);
                            }
                        }
                        width += elemWidth;
                    } else if (isa<llvm::ArrayType>(**elem)) {
                        auto *selem = dyn_cast<llvm::ArrayType>(*elem);
                        unsigned elemWidth;
                        if (arcGen) elemWidth = getArrayWidthArcGen((*selem), width);
                        else elemWidth = getArrayWidth((*selem), width);
                        width = elemWidth;
                    } else if (isa<llvm::StructType>(**elem)) {
                        auto *selem = dyn_cast<llvm::StructType>(*elem);
                        unsigned elemWidth = getStructWidth((*selem), width, arcGen);
                        width = elemWidth;
                    }
                }
                if (arcGen) {
                    if (fieldIndex->find(width) == fieldIndex->end()) {
                        (*fieldIndex)[width] = new std::vector<unsigned>();
                    }
                }
                return width;
            }

            unsigned getTypeBitWidth(Type *ty) {
                unsigned size;
                if (ty->isIntegerTy()) {
                    size = ty->getIntegerBitWidth();
                } else if (ty->isStructTy()) {
                    StructType *sty = dyn_cast<StructType>(ty);
                    size = getStructWidth(*sty, 0, false);
                } else if (ty->isArrayTy()) {
                    ArrayType *aty = dyn_cast<ArrayType>(ty);
                    size = getArrayWidth(*aty, 0);
                }
                return size;
            }

            std::vector<Value*>* getBFCOutputs(Instruction *ii) {
                if (isa<llvm::CallInst>(&*ii)) {
                    MDNode *PrimateMetadata = ii->getMetadata("primate");
                    if (PrimateMetadata != NULL) {
                        if (cast<MDString>(PrimateMetadata->getOperand(0))->getString() == "blue") {
                            auto numIn_i = cast<ConstantInt>(cast<ConstantAsMetadata>(PrimateMetadata->getOperand(3))->getValue())->getValue();
                            unsigned numIn = unsigned(numIn_i.getZExtValue());
                            if (numIn < (ii->getNumOperands()-1)) { // last operand is always metadata
                                std::vector<Value*> *outList = new std::vector<Value*>();
                                for (int i = numIn; i < (ii->getNumOperands()-1); i++) {
                                    outList->push_back(ii->getOperand(i));
                                }
                                return outList;
                            }
                        }
                    }
                }
                return NULL;
            }

            std::vector<Value*>* getBFCInputs(Instruction *ii) {
                if (isa<llvm::CallInst>(&*ii)) {
                    MDNode *PrimateMetadata = ii->getMetadata("primate");
                    if (PrimateMetadata != NULL) {
                        if (cast<MDString>(PrimateMetadata->getOperand(0))->getString() == "blue") {
                            auto numIn_i = cast<ConstantInt>(cast<ConstantAsMetadata>(PrimateMetadata->getOperand(3))->getValue())->getValue();
                            unsigned numIn = unsigned(numIn_i.getZExtValue());
                            if (numIn > 0) {
                                std::vector<Value*> *inList = new std::vector<Value*>();
                                for (int i = 0; i < numIn; i++) {
                                    inList->push_back(ii->getOperand(i));
                                }
                                return inList;
                            }
                        }
                    }
                }
                return NULL;
            }

            bool checkMemAlias(Value *ptr0, unsigned size0, Value *ptr1, unsigned size1) {
                if (pointerMap->find(ptr0) == pointerMap->end()) {
                    errs() << "pointer0 not initialized\n";
                    ptr0->print(errs());
                    errs() << "\n";
                    exit(1);
                }
                if (pointerMap->find(ptr1) == pointerMap->end()) {
                    errs() << "pointer1 not initialized\n";
                    ptr1->print(errs());
                    errs() << "\n";
                    exit(1);
                }
                Value* base0 = (*pointerMap)[ptr0]->base;
                Value* base1 = (*pointerMap)[ptr1]->base;
                unsigned offset0 = (*pointerMap)[ptr0]->offset;
                unsigned offset1 = (*pointerMap)[ptr1]->offset;
                if (base0 == base1) {
                    if (!((offset0 + size0 <= offset1) || (offset0 >= offset1 + size1))) {
                        return true;
                    }
                }
                return false;
            }

            bool isReachable(Value* src, std::set<Value*> &dst) {
                // simple DFS
                std::vector<Value*> stack;
                for (auto it = dependencyForest[src]->begin(); it != dependencyForest[src]->end(); it++) {
                    stack.push_back(it->first);
                }
                while (!stack.empty()) {
                    Value *inst = stack.back();
                    if (dst.find(inst) != dst.end()) {
                        return true;
                    } else {
                        stack.pop_back();
                        for (auto it = dependencyForest[inst]->begin(); it != dependencyForest[inst]->end(); it++) {
                            stack.push_back(it->first);
                        }
                    }
                }
                return false;
            }

            inline void memInstAddRAWDep(Instruction* inst, Value* srcPtr, unsigned size, ValueMap<Value*, std::vector<std::pair<Value*, unsigned>>> &storeInsts) {
                for (auto si = storeInsts.begin(); si != storeInsts.end(); si++) {
                    // check all instructions that write memory
                    // si->first->print(errs());
                    // errs() << ":\n";
                    bool isAlias = false;
                    for (auto sp = si->second.rbegin(); sp != si->second.rend(); sp++) {
                        if (checkMemAlias(srcPtr, size, sp->first, sp->second)) {
                            // RAW dependency
                            isAlias = true;
                            // (*dependencyForest[&*inst])[si->first] = true;
                            break;
                        }
                    }
                    if (isAlias) {
                        // Only add to the dependency list if it's an immediate dependency
                        std::set<Value*> dst{si->first};
                        bool immDep = true;
                        for (auto dep = dependencyForest[&*inst]->begin(); dep != dependencyForest[&*inst]->end(); dep++) {
                            if (dep->second && (isReachable(dep->first, dst))) {
                                immDep = false;
                                break;
                            }
                        }
                        if (immDep) (*dependencyForest[&*inst])[si->first] = true;
                    }
                }
            }

            inline void memInstAddWARDep(Instruction* inst, Value* dstPtr, unsigned size, ValueMap<Value*, std::vector<std::pair<Value*, unsigned>>> &loadInsts) {
                for (auto li = loadInsts.begin(); li != loadInsts.end(); li++) {
                    // check all instructions that read memory
                    // li->first->print(errs());
                    // errs() << ":\n";
                    for (auto lp = li->second.begin(); lp != li->second.end(); lp++) {
                        if (checkMemAlias(dstPtr, size, lp->first, lp->second)) {
                            // WAR dependency
                            dependencyForest[&*inst]->insert({li->first, false});
                        }
                    }
                }
            }

            void initializeDependencyForest(Function &F) {
                ValueMap<Value*, std::vector<std::pair<Value*, unsigned>>> loadInsts;
                ValueMap<Value*, std::vector<std::pair<Value*, unsigned>>> storeInsts;
                dependencyForest.clear();
                loadMergedInst.clear();
                for (Function::iterator bi = F.begin(), be = F.end(); bi != be; bi++) {
                    BasicBlock *bb = &*bi;
                    loadInsts.clear();
                    storeInsts.clear();
                    for (BasicBlock::iterator ii = bb->begin(), ie = bb->end(); ii != ie; ii++) {
                        Instruction *inst = &*ii;
                        // inst->print(errs());
                        // errs() << ":\n";
                        if (isa<LoadInst>(*inst)) {
                            // inst->print(errs());
                            // errs() << ":\n";
                            dependencyForest[&*inst] = new std::map<Value*, bool>();
                            Value* srcPtr = inst->getOperand(0);
                            Type *loadType = inst->getType();
                            unsigned size = getTypeBitWidth(loadType);
                            loadInsts[&*inst].push_back({srcPtr, size});
                            memInstAddRAWDep(inst, srcPtr, size, storeInsts);
                        } else if (isa<StoreInst>(*inst)) {
                            // inst->print(errs());
                            // errs() << ":\n";
                            dependencyForest[&*inst] = new std::map<Value*, bool>();
                            auto *tmp = dyn_cast<llvm::StoreInst>(&*inst);
                            Value *srcOp = tmp->getValueOperand();
                            Value *ptrOp = tmp->getPointerOperand();
                            if (isa<Instruction>(*srcOp)) {
                                Instruction *op_inst = dyn_cast<Instruction>(srcOp);
                                if (op_inst->getParent() == bb && (!isa<PHINode>(*op_inst)))
                                    (*dependencyForest[&*inst])[srcOp] = true;
                            }
                            unsigned size = getTypeBitWidth(srcOp->getType());
                            storeInsts[&*inst].push_back({ptrOp, size});
                            memInstAddWARDep(inst, ptrOp, size, loadInsts);
                        } else if (isa<CallInst>(*inst)) {
                            auto *tmp = dyn_cast<llvm::CallInst>(&*inst);
                            Function* foo = tmp->getCalledFunction();
                            if (foo->getName().contains("memcpy")) {
                                dependencyForest[&*inst] = new std::map<Value*, bool>();
                                Value *dstPtr = tmp->getOperand(0);
                                Value *srcPtr = tmp->getOperand(1);
                                Value *size = tmp->getOperand(2);
                                if (!isa<ConstantInt>(size)) {
                                    errs() << "Error: memcpy does not have constant size\n";
                                    inst->print(errs());
                                    errs() << "\n";
                                    exit(1);
                                }
                                auto *size_const = dyn_cast<ConstantInt>(size);
                                auto size_val = size_const->getValue();
                                int size_u = int(size_val.getSExtValue());
                                memInstAddRAWDep(inst, srcPtr, size_u, storeInsts);
                                memInstAddWARDep(inst, dstPtr, size_u, loadInsts);
                                loadInsts[&*inst].push_back({srcPtr, size_u*8});
                                storeInsts[&*inst].push_back({dstPtr, size_u*8});
                            // } else if (isBlueCall(inst)) {
                            //     dependencyForest[&*inst] = new std::map<Value*, bool>();
                            //     std::vector<std::pair<Value*, unsigned>> inOps;
                            //     std::vector<std::pair<Value*, unsigned>> outOps;
                            //     auto inList = getBFCInputs(inst);
                            //     if (inList != NULL) {
                            //         for (auto op = inList->begin(); op != inList->end(); op++) {
                            //             Type* op_type = (*op)->getType();
                            //             if (op_type->isPointerTy()) {
                            //                 Value *srcPtr = *op;
                            //                 PointerType *ptrType = dyn_cast<PointerType>(op_type);
                            //                 Type *pteType = ptrType->getElementType();
                            //                 unsigned size = getTypeBitWidth(pteType);
                            //                 inOps.push_back({srcPtr, size});
                            //                 memInstAddRAWDep(inst, srcPtr, size, storeInsts);
                            //             }
                            //         }
                            //     }
                            //     auto outList = getBFCOutputs(inst);
                            //     if (outList != NULL) {
                            //         for (auto op = outList->begin(); op != outList->end(); op++) {
                            //             Type* op_type = (*op)->getType();
                            //             if (op_type->isPointerTy()) {
                            //                 Value *dstPtr = *op;
                            //                 PointerType *ptrType = dyn_cast<PointerType>(op_type);
                            //                 Type *pteType = ptrType->getElementType();
                            //                 unsigned size = getTypeBitWidth(pteType);
                            //                 outOps.push_back({dstPtr, size});
                            //                 memInstAddWARDep(inst, dstPtr, size, loadInsts);
                            //             }
                            //         }
                            //     }
                            //     if (inList != NULL)
                            //         loadInsts[&*inst].insert(loadInsts[&*inst].end(), inOps.begin(), inOps.end());
                            //     if (outList != NULL)
                            //         storeInsts[&*inst].insert(storeInsts[&*inst].end(), outOps.begin(), outOps.end());
                            }
                        } else if (!(isa<GetElementPtrInst>(*inst) || isa<BitCastInst>(*inst) || isa<AllocaInst>(*inst) || isa<PHINode>(*inst))) {
                            dependencyForest[&*inst] = new std::map<Value*, bool>();
                            for (auto OI = inst->op_begin(); OI != inst->op_end(); ++OI) {
                                if (isa<Instruction>(*OI)) {
                                    Instruction* op_inst = dyn_cast<Instruction>(OI);
                                    if (op_inst->getParent() == bb && (!isa<PHINode>(*op_inst)))
                                        (*dependencyForest[&*inst])[&*op_inst] = true;
                                }
                            }
                        }
                    }
                }
            }

            void mergeExtInstructions() {
                for (auto it = dependencyForest.begin(); it != dependencyForest.end(); it++) {
                    for (auto dep = it->second->begin(); dep != it->second->end();) {
                        Value* dep_inst = dep->first;
                        if (isa<ZExtInst>(*dep_inst) || isa<SExtInst>(*dep_inst)) {
                            Value* new_dep = dependencyForest[dep_inst]->begin()->first;
                            bool rel = dep->second;
                            // errs() << "start erase\n";
                            // errs() << "erase success\n";
                            if (new_dep != NULL) {
                                if (rel)
                                    (*(it->second))[new_dep] = rel;
                                else
                                    it->second->insert({new_dep, rel});
                            }
                            // errs() << "insert success\n";
                            it->second->erase(dep++);
                        } else {
                            ++dep;
                        }
                    }
                }
                for (auto it = dependencyForest.begin(); it != dependencyForest.end();) {
                    Value *inst = it->first;
                    if (isa<ZExtInst>(*inst) || isa<SExtInst>(*inst)) {
                        dependencyForest.erase(it++);
                    } else {
                        ++it;
                    }
                }
            }

            void mergeLoadInstructions() {
                ValueMap<Value*, std::set<Value*>> loadRAWDependents;
                ValueMap<Value*, std::set<Value*>> loadWARDependents;
                ValueMap<Value*, bool> loadMergeable;
                unmergeableLoad.clear();
                for (auto it = dependencyForest.begin(); it != dependencyForest.end(); it++) {
                    if (isa<LoadInst>(*(it->first))) {
                        loadMergeable[it->first] = true;
                    } else {
                        for (auto dep = it->second->begin(); dep != it->second->end(); dep++) {
                            if (isa<LoadInst>(*(dep->first))) {
                                if (dep->second)
                                    loadRAWDependents[dep->first].insert(it->first);
                                else
                                    loadWARDependents[dep->first].insert(it->first);
                            }
                        }
                    }
                }
                for (auto it = loadRAWDependents.begin(); it != loadRAWDependents.end(); it++) {
                    // it->first->print(errs());
                    // errs() << "\n";
                    for (auto dep = it->second.begin(); dep != it->second.end(); dep++) {
                        if (loadWARDependents.find(it->first) != loadWARDependents.end()) {
                            if (isReachable(*dep, loadWARDependents[it->first])) {
                                // load is not mergeable if any of its RAW dependents depend on its WAR dependents
                                loadMergeable[it->first] = false;
                                unmergeableLoad.insert(it->first);
                                break;
                            }
                        }
                    }
                }
                for (auto it = loadMergeable.begin(); it != loadMergeable.end(); it++) {
                    if (it->second) {
                        // errs() << "load mergeable\n";
                        auto newDepList = dependencyForest[it->first];
                        for (auto inst_it = loadRAWDependents[it->first].begin(); inst_it != loadRAWDependents[it->first].end(); inst_it++) {
                            loadMergedInst.insert(*inst_it);
                            int erase_count = dependencyForest[*inst_it]->erase(it->first);
                            // (*inst_it)->print(errs());
                            // errs() << ": erase " << erase_count << "\n";
                            for (auto newDep = newDepList->begin(); newDep != newDepList->end(); newDep++) {
                                (*dependencyForest[*inst_it])[newDep->first] = newDep->second;
                            }
                        }
                        for (auto inst_it = loadWARDependents[it->first].begin(); inst_it != loadWARDependents[it->first].end(); inst_it++) {
                            dependencyForest[*inst_it]->erase(it->first);
                        }
                        dependencyForest.erase(it->first);
                    }
                }
            }

            void mergeStoreInstructions() {
                ValueMap<Value*, Value*> storeMap;
                unmergeableStore.clear();
                for (auto it = dependencyForest.begin(); it != dependencyForest.end();) {
                    if(isa<StoreInst>(*(it->first))) {
                        std::set<Value*> storeRAWDependents;
                        std::set<Value*> storeWARDependents;
                        Value* storeSrc;
                        for (auto dep = it->second->begin(); dep != it->second->end(); dep++) {
                            if (dep->second) {
                                storeSrc = dep->first;
                                storeRAWDependents.insert(dep->first);
                            } else {
                                storeWARDependents.insert(dep->first);
                            }
                        }
                        bool mergeable = true;
                        if (storeRAWDependents.empty()) mergeable = false;
                        for (auto dep = storeWARDependents.begin(); dep != storeWARDependents.end(); dep++) {
                            if (isReachable(*dep, storeRAWDependents)) {
                                mergeable = false;
                                break;
                            }
                        }
                        if (mergeable) {
                            for (auto dep = storeWARDependents.begin(); dep != storeWARDependents.end(); dep++) {
                                dependencyForest[storeSrc]->insert({*dep, false});
                            }
                            storeMap[it->first] = storeSrc;
                            dependencyForest.erase(it++);
                        } else {
                            unmergeableStore.insert(it->first);
                            ++it;
                        }
                    } else {
                        ++it;
                    }
                }
                for (auto it = dependencyForest.begin(); it != dependencyForest.end(); it++) {
                    for (auto dep = it->second->begin(); dep != it->second->end();) {
                        if (storeMap.find(dep->first) != storeMap.end()) {
                            // depend on a mergeable store instruction
                            Value* newDep = storeMap[dep->first];
                            bool depType = dep->second;
                            if (depType)
                                (*(it->second))[newDep] = depType;
                            else
                                it->second->insert({newDep, depType});
                            it->second->erase(dep++);
                        } else {
                            ++dep;
                        }
                    }
                }
            }

            void buildDependencyForest(Function &F) {
                initializeDependencyForest(F);
                mergeExtInstructions();
                mergeLoadInstructions();
                mergeStoreInstructions();
            }

            void InitializePointerMap(Function &F) {
                pointerMap = new ValueMap<Value*, ptrInfo_t*>();
                for (Function::arg_iterator arg = F.arg_begin(); arg != F.arg_end(); ++arg){
                    const Type* arg_type = arg->getType();
                    if (arg_type->isPointerTy()) {
                        (*pointerMap)[&*arg] = new ptrInfo_t(&*arg, 0);
                    }
                }
                for (inst_iterator instruction = inst_begin(F), e = inst_end(F); instruction != e; ++instruction) {
                    // instruction->print(errs());
                    // errs() << ":\n";
                    if (isa<AllocaInst>(*instruction)) {
                        (*pointerMap)[&*instruction] = new ptrInfo_t(&*instruction, 0);
                    } else if (isa<GetElementPtrInst>(*instruction)) {
                        auto *inst = dyn_cast<llvm::GetElementPtrInst>(&*instruction);
                        unsigned offset = 0;
                        Value *basePtr = inst->getPointerOperand();
                        if (pointerMap->find(basePtr) == pointerMap->end()) continue;
                        if ((*pointerMap)[basePtr]->base != basePtr) {
                            offset = (*pointerMap)[basePtr]->offset;
                            basePtr = (*pointerMap)[basePtr]->base;
                        }
                        Type *type = inst->getSourceElementType();
                        int i = 0;
                        for (auto idx = inst->idx_begin(); idx != inst->idx_end(); idx++) {
                            if (isa<ConstantInt>(*idx)) {
                                auto *idx_const = dyn_cast<ConstantInt>(idx);
                                auto idx_val = idx_const->getValue();
                                int idx_u = int(idx_val.getSExtValue());
                                if (isa<StructType>(*type)) {
                                    auto stype = dyn_cast<StructType>(type);
                                    if (i == 0) {
                                        unsigned elemWidth = getStructWidth(*stype, 0, false);
                                        offset += idx_u * elemWidth;
                                    } else {
                                        int j = 0;
                                        Type *tmp;
                                        for (auto elem = stype->element_begin(); elem != stype->element_end(), j <= idx_u; elem++, j++) {
                                            if (j == idx_u) {
                                                tmp = (*elem);
                                                break;
                                            }
                                            offset += getTypeBitWidth(*elem);
                                        }
                                        type = tmp;
                                    }
                                    i = 1;
                                } else if (isa<ArrayType>(*type)) {
                                    auto *atype = dyn_cast<llvm::ArrayType>(type);
                                    if (i == 0) {
                                        unsigned elemWidth = getArrayWidth(*atype, 0);
                                        offset += idx_u * elemWidth;
                                    } else {
                                        Type *elem = atype->getElementType();
                                        unsigned elemWidth = getTypeBitWidth(elem);
                                        offset += (idx_u * elemWidth);
                                        type = elem;
                                    }
                                    i = 1;
                                } else {
                                    errs() << "Error: undefined type: ";
                                    inst->print(errs());
                                    errs() << "\n";
                                    exit(1);
                                }
                            } else {
                                errs() << "Error: pointer is not constant: ";
                                inst->print(errs());
                                errs() << "\n";
                                exit(1);
                            }
                        }
                        (*pointerMap)[&*instruction] = new ptrInfo_t(basePtr, offset);
                        // errs() << offset << "\n";
                    } else if (isa<BitCastInst>(*instruction)) {
                        Value *srcOp = instruction->getOperand(0);
                        (*pointerMap)[&*instruction] = (*pointerMap)[&*srcOp];
                    }
                }
            }

            virtual void InitializeAliasMap(Function &F) {
                aliasMap = new ValueMap<Value*, Value*>();
                for (Function::arg_iterator arg = F.arg_begin(); arg != F.arg_end(); ++arg){
                    (*aliasMap)[&*arg] = &*arg;
                }

                for (inst_iterator instruction = inst_begin(F), e = inst_end(F); instruction != e; ++instruction) {
                    Value *srcOp;
                    if (isa<GetElementPtrInst>(*instruction)) {
                        auto *inst = dyn_cast<llvm::GetElementPtrInst>(&*instruction);
                        if (isa<StructType>(*(inst->getSourceElementType()))) {
                            auto *type = dyn_cast<StructType>(inst->getSourceElementType());
                            if (!(type->isLiteral())) {
                                // getelementptr instruction for a struct
                                srcOp = instruction->getOperand(0);
                                (*aliasMap)[&*instruction] = (*aliasMap)[&*srcOp];
                                // errs() << "getelementptr " << srcOp->getName() << "\n";
                            } else {
                                (*aliasMap)[&*instruction] = &*instruction;
                            }
                        } else {
                            (*aliasMap)[&*instruction] = &*instruction;
                        }
                    } else if (isa<LoadInst>(*instruction)) {
                        srcOp = instruction->getOperand(0);
                        if (unmergeableLoad.find(&*instruction) == unmergeableLoad.end()) {
                            (*aliasMap)[&*instruction] = (*aliasMap)[&*srcOp];
                        } else {
                            (*aliasMap)[&*instruction] = &*instruction;
                        }
                        // errs() << "load " << srcOp->getName() << "\n";
                    } else if (isa<StoreInst>(*instruction)) {
                        auto *inst = dyn_cast<llvm::StoreInst>(&*instruction);
                        srcOp = inst->getValueOperand();
                        Value *ptrOp = inst->getPointerOperand();
                        if (unmergeableStore.find(&*instruction) == unmergeableStore.end()) {
                            (*aliasMap)[srcOp] = (*aliasMap)[ptrOp];
                            // errs() << "store" << ptrOp->getName() << "\n";
                        }
                    } else if (isa<BitCastInst>(*instruction)) {
                        srcOp = instruction->getOperand(0);
                        (*aliasMap)[&*instruction] = (*aliasMap)[&*srcOp];
                        // errs() << "bitcast " << srcOp->getName() << "\n";
                    } else if (isa<ZExtInst>(*instruction)) {
                        srcOp = instruction->getOperand(0);
                        (*aliasMap)[&*instruction] = (*aliasMap)[&*srcOp];
                        // errs() << "zext " << srcOp->getName() << "\n";
                    } else {
                        (*aliasMap)[&*instruction] = &*instruction;
                    }
                }
            }

            void initializeVarNameMap(Function &F) {
                int i = 1;
                for (inst_iterator inst_it = inst_begin(F), e = inst_end(F); inst_it != e; ++inst_it) {
                    if (isa<DbgDeclareInst>(*inst_it)) {
                        auto* dbgInst = dyn_cast<DbgDeclareInst>(&*inst_it);
                        auto* instMeta = cast<MetadataAsValue>(dbgInst->getOperand(0))->getMetadata();
                        Value* inst = cast<ValueAsMetadata>(instMeta)->getValue();
                        auto *varMeta = cast<DILocalVariable>(cast<MetadataAsValue>(dbgInst->getOperand(1))->getMetadata());
                        std::string name = (varMeta->getName()).str();
                        varNameMap[inst] = name;
                        // errs() << "name: " << name << "\n";
                        std::vector<int> fieldWidth;
                        if (isa<AllocaInst>(*inst)) {
                            auto* aInst = dyn_cast<AllocaInst>(inst);
                            Type *aType = aInst->getAllocatedType();
                            if (isa<StructType>(*aType)) {
                                auto stype = dyn_cast<StructType>(aType);
                                for (auto elem = stype->element_begin(); elem != stype->element_end(); elem++) {
                                    if (isa<IntegerType>(**elem)) {
                                        unsigned elemWidth = (*elem)->getIntegerBitWidth();
                                        fieldWidth.emplace_back(elemWidth);
                                    } else {
                                        errs() << "Each field must be integer type\n";
                                        exit(1);
                                    }
                                }
                            }
                        }
                        if (name != "top_intf") {
                            varRegMap[inst] = i;
                            i++;

                            // record the types
                            DIType* varType = varMeta->getType();
                            DICompositeType* varBaseType;
                            std::string varTypeName;
                            if (isa<DIDerivedType>(*varType)) {
                                varTypeName = ((cast<DIDerivedType>(varType))->getName()).str();
                                auto *tmp = (cast<DIDerivedType>(varType))->getBaseType();
                                if (isa<DICompositeType>(*tmp)) {
                                    varBaseType = cast<DICompositeType>(tmp);
                                } else {
                                    continue;
                                }
                            } else if (isa<DICompositeType>(*varType)) {
                                varBaseType = cast<DICompositeType>(varType);
                                varTypeName = (varBaseType->getName()).str();
                            } else {
                                continue;
                            }
                            if (varTypeMap.find(varTypeName) == varTypeMap.end()) {
                                if (varBaseType->getTag() == 0x0013) {  // The ID of DW_TAG_structure_type
                                    varTypeMap[varTypeName] = new std::vector<std::pair<std::string, int>>();
                                    DINodeArray fieldArray = varBaseType->getElements();
                                    int j = 0;
                                    for (auto field_it = fieldArray.begin(); field_it != fieldArray.end(); field_it++) {
                                        if (isa<DIDerivedType>(**field_it)) {
                                            DIDerivedType* field = cast<DIDerivedType>(*field_it);
                                            if (field->getTag() == 0x000d) { // The ID of DW_TAG_member
                                                auto fieldName = field->getName();
                                                varTypeMap[varTypeName]->emplace_back(std::make_pair(fieldName, fieldWidth[j]));
                                                j++;
                                                // errs() << "Type: " << varTypeName << ", Field: " << fieldName << ", Size: " << field->getSizeInBits() << "\n";
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            void initializeVarNames(Function &F) {
                InitializePointerMap(F);
                buildDependencyForest(F);
                InitializeAliasMap(F);
                initializeVarNameMap(F);
            }

            virtual bool livenessAnalysis(Function &F) {
                domain.clear();
                bvIndexToInstrArg = new std::vector<Value*>();
                valueToBitVectorIndex = new ValueMap<Value*, int>();
                instrInSet = new ValueMap<const Instruction*, BitVector*>();

                int index = 0;
                for (Function::arg_iterator arg = F.arg_begin(); arg != F.arg_end(); ++arg){
                    domain.push_back(&*arg);
                    bvIndexToInstrArg->push_back(&*arg);
                    (*valueToBitVectorIndex)[&*arg] = index;
                    index++;
                }

                for (inst_iterator instruction = inst_begin(F), e = inst_end(F); instruction != e; ++instruction) {
                    domain.push_back(&*instruction);
                    bvIndexToInstrArg->push_back(&*instruction);
                    (*valueToBitVectorIndex)[&*instruction] = index;
                    index++;
                }

                domainSize = domain.size();

                //initialize the IN set set inside the block for each instruction.     
                for (inst_iterator instruction = inst_begin(F), e = inst_end(F); instruction != e; ++instruction) {
                    (*instrInSet)[&*instruction] = new BitVector(domainSize, false); 
                }
                //call the backward analysis method in dataflow
                DataFlow<BitVector>::runOnFunction(F);
                // F.print(errs(), this);
                
                //compute the histogram
                // for(inst_iterator instruction = inst_begin(F), e = inst_end(F); instruction != e; ++instruction) {
                //     calculate(&*instruction);
                // }

                return false;
            }

            BitVector* getLiveoutVariables(std::set<BasicBlock*> &BBset, BasicBlock &endBB) {
                BitVector* updated = new BitVector(domainSize, false);
                for (auto bb_it = BBset.begin(); bb_it != BBset.end(); bb_it++) {
                    for (auto inst_it = (*bb_it)->begin(); inst_it != (*bb_it)->end(); inst_it++) {
                        Instruction *tempInst = &*inst_it;
                        if (isa<CallInst>(*tempInst)) {
                            // input header
                            auto *inst = dyn_cast<llvm::CallInst>(&*tempInst);
                            Function* foo = inst->getCalledFunction();
                            if (foo->getName().contains("Input_header")) {
                                Value* ptrOp = inst->getOperand(2);
                                (*updated)[(*valueToBitVectorIndex)[(*aliasMap)[ptrOp]]] = true;
                                // tempInst->print(errs());
                            } else if (foo->getName().contains("Input_done")) {
                                break;
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

                (*updated) &= (*(*out)[&endBB]);

                return updated;
            }

            void initializeBBLoc(Function &F) {
                for (auto bb_it = F.begin(); bb_it != F.end(); bb_it++) {
                    for (auto inst_it = bb_it->begin(); inst_it != bb_it->end(); inst_it++) {
                        Instruction *inst = dyn_cast<Instruction>(&*inst_it);
                        if (DILocation *Loc = inst->getDebugLoc()) {
                            unsigned line = Loc->getLine();
                            unsigned col = Loc->getColumn();
                            if (lineToBBs.find(line) != lineToBBs.end()) {
                                bool found = false;
                                for (auto it = lineToBBs[line]->begin(); it != lineToBBs[line]->end(); it++) {
                                    if (it->second == &*bb_it) {
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found) {
                                    (*lineToBBs[line])[col] = &*bb_it;
                                }
                            } else {
                                lineToBBs[line] = new std::map<int, Value*>();
                                (*lineToBBs[line])[col] = &*bb_it;
                            }
                        }
                    }
                }
            }

            BasicBlock* getBBfromLoc(int lineStart, int colStart, int lineEnd, int colEnd) {
                if (lineToBBs.find(lineStart) != lineToBBs.end()) {
                    for (auto it = lineToBBs[lineStart]->begin(); it != lineToBBs[lineStart]->end(); it++) {
                        if (it->first >= colStart) {
                            return dyn_cast<BasicBlock>(it->second);
                        }
                    }
                }
                for (int line = lineStart+1; line < lineEnd; line++) {
                    if (lineToBBs.find(line) != lineToBBs.end()) {
                        for (auto it = lineToBBs[line]->begin(); it != lineToBBs[line]->end(); it++) {
                            return dyn_cast<BasicBlock>(it->second);
                        }
                    }
                }
                if (lineToBBs.find(lineEnd) != lineToBBs.end()) {
                    for (auto it = lineToBBs[lineEnd]->begin(); it != lineToBBs[lineEnd]->end(); it++) {
                        if (it->first < colEnd) {
                            return dyn_cast<BasicBlock>(it->second);
                        }
                    }
                }
                return NULL;
            }

            int initializeProfData() {
                std::ifstream file("prof.json");
                std::string json((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());

                rapidjson::Document doc;
                doc.Parse(json.c_str());
             
                if (doc.HasParseError()) {
                    std::cerr << "Error parsing JSON: "
                         << doc.GetParseError() << std::endl;
                    return 1;
                }

                // parse branch regions
                const rapidjson::Value &branches = doc["data"][0]["files"][0]["branches"];
                numProfBranches = branches.Size();
                profBranches = new int*[numProfBranches];
                for (int i = 0; i < numProfBranches; i++) {
                    profBranches[i] = new int[9];
                    for (int j = 0; j < 9; j++) {
                        profBranches[i][j] = branches[i][j].GetInt();
                    }
                }

                // parse segments
                filename = doc["data"][0]["files"][0]["filename"].GetString();
                const rapidjson::Value &segments = doc["data"][0]["files"][0]["segments"];

                numProfSegments = segments.Size();
                profSegments = new int*[numProfSegments];
                for (int i = 0; i < numProfSegments; i++) {
                    profSegments[i] = new int[6];
                    for (int j = 0; j < 3; j++) {
                        profSegments[i][j] = segments[i][j].GetInt();
                    }
                    for (int j = 3; j < 6; j++) {
                        profSegments[i][j] = segments[i][j].GetBool();
                    }
                }
                numLine = segments[numProfSegments-1][0].GetInt();

                // parse regions
                const rapidjson::Value &functions = doc["data"][0]["functions"];
                for (rapidjson::Value::ConstValueIterator function_it = functions.Begin(); function_it != functions.End(); function_it++) {
                    std::string func_name = (*function_it)["name"].GetString();
                    if (func_name.find("primate_main") != std::string::npos) {
                        const rapidjson::Value &regions = (*function_it)["regions"];
                        numProfRegions = regions.Size();
                        profRegions = new int*[numProfRegions];
                        for (int i = 0; i < numProfRegions; i++) {
                            profRegions[i] = new int[8];
                            for (int j = 0; j < 8; j++) {
                                profRegions[i][j] = regions[i][j].GetInt();
                            }
                        }
                        break;
                    }
                }

                return 0;
            }

            int getSegExeCount(const unsigned line, const unsigned col, int &segId, int &segLineStart, int &segColStart, int &segLineEnd, int &segColEnd) {
                int segExeCount = 0;
                for (int i = 0; i < numProfSegments; i++) {
                    if (profSegments[i][3] == 0) {
                        // skip this entry if it has no count
                        continue;
                    }
                    segLineStart = profSegments[i][0];
                    segColStart = profSegments[i][1];
                    if ((line < segLineStart) || (line == segLineStart && col < segColStart)) {
                        continue;
                    } else if (i+1 < numProfSegments) {
                        if (line > profSegments[i+1][0] || (line == profSegments[i+1][0] && col >= profSegments[i+1][1])) {
                            continue;
                        }
                    }

                    segExeCount = profSegments[i][2];
                    segId = i;
                    for (int j = i+1; j < numProfSegments; j++) {
                        if (profSegments[j][3] == 0) {
                            // skip this entry if it has no count
                            continue;
                        }
                        segLineEnd = profSegments[j][0];
                        segColEnd = profSegments[j][1];
                        break;
                    }
                    break;
                }
                return segExeCount;
            }

            Function* initializeExeCount(Module &M) {
                numInstPerSeg = new int[numProfSegments];
                for (int i = 0; i < numProfSegments; i++) {
                    numInstPerSeg[i] = 0;
                }
                for (Module::iterator mi = M.begin(); mi != M.end(); mi++) {
                    if ((mi->getName()).find("primate_main") != StringRef::npos) {
                        numTotalInst = 0;
                        numTotalExeCount = 0;
                        int profSegExeCount;
                        int profSegLineStart = 0;
                        int profSegColStart = 0;
                        int profSegLineEnd = 0;
                        int profSegColEnd = 0;
                        int segId = 0;
                        for (Function::iterator bi = mi->begin(); bi != mi->end(); bi++) {
                            BasicBlock *bb = &*bi;
                            bool newBB = true;
                            int numInstWODbg = 0;
                            for (auto ii = bb->begin(); ii != bb->end(); ++ii) {
                                Instruction *inst = dyn_cast<Instruction>(&*ii);
                                if (!(isa<CallInst>(*inst) || isa<AllocaInst>(*inst) || isa<PHINode>(*inst))) {
                                    if (DILocation *Loc = inst->getDebugLoc()) {
                                        unsigned line = Loc->getLine();
                                        unsigned col = Loc->getColumn();
                                        numTotalInst++;
                                        if ((line < profSegLineStart) || (line == profSegLineStart && col < profSegColStart) || (line == profSegLineEnd && col >= profSegColEnd) || (line > profSegLineEnd)) {
                                            // Update the count if the instruction is in a new segment
                                            profSegExeCount = getSegExeCount(line, col, segId, profSegLineStart, profSegColStart, profSegLineEnd, profSegColEnd);
                                        }
                                        numInstPerSeg[segId] += (numInstWODbg + 1);
                                        numTotalExeCount += (profSegExeCount * (numInstWODbg + 1));
                                        numInstWODbg = 0;
                                        newBB = false;
                                    } else if (newBB) {
                                        numTotalInst++;
                                        numInstWODbg++;
                                    } else {
                                        numTotalInst += 1;
                                        numInstPerSeg[segId] += 1;
                                        numTotalExeCount += profSegExeCount;
                                    }
                                }
                            }
                        }
                        avgExeCountPerInst = numTotalExeCount / numTotalInst;
                        errs() << "Total static count: " << numTotalInst << ", total dynamic count: " << numTotalExeCount << "\n";
                        return (&*mi);
                    }
                }
                return NULL;
            }

            bool getSource(std::ifstream &inFile, std::string &sourceBuf, int &line, int &col, const int lineEnd, const int colEnd, std::string pattern) {
                // Return true if the pattern is found
                char c;

                while ((line < lineEnd) || (line == lineEnd && col < colEnd)) {
                    inFile.get(c);
                    if (c == '\n') {
                        line++;
                        col = 1;
                    } else {
                        col++;
                    }
                    sourceBuf += c;
                }

                if (sourceBuf.find(pattern) != std::string::npos) {
                    return true;
                }

                return false;
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

            std::string insertExit(std::set<BasicBlock*> &BBset, BasicBlock* startBB, BasicBlock* endBB, BasicBlock* doneBB, int btID) {
                std::set<BasicBlock*> intersect;
                std::set<BasicBlock*> reachable;
                std::vector<BasicBlock*> workstack;
                std::vector<BasicBlock*> path;
                ValueMap<BasicBlock*, bool> visited;

                visited[doneBB] = true;
                reachable.insert(endBB);
                workstack.emplace_back(startBB);
                while (!workstack.empty()) {
                    BasicBlock *BB = workstack.back();
                    workstack.pop_back();
                    visited[BB] = true;

                    path.emplace_back(BB);

                    for (auto succ = succ_begin(BB), succEnd = succ_end(BB); succ != succEnd; ++succ) {
                        BasicBlock* bb_succ = *succ;
                        if (reachable.find(bb_succ) != reachable.end()) {
                            for (auto it = path.begin(); it != path.end(); it++) {
                                reachable.insert(*it);
                            }
                            path.clear();
                            continue;
                        } else if (!visited[bb_succ]) {
                            workstack.emplace_back(bb_succ);
                        }
                    }
                }

                for (auto bb_it = reachable.begin(); bb_it != reachable.end(); bb_it++) {
                    if (BBset.find(*bb_it) != BBset.end()) {
                        intersect.insert(*bb_it);
                    }
                }

                BitVector* liveout = getLiveoutVariables(intersect, *endBB);

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
                
                return exitCode;
            }

            bool checkRegionEntry(int segId) {
                if (profSegments[segId][4] != 0) {
                    return true;
                } else if (segId > 0 && (profSegments[segId-1][0] == profSegments[segId][0]) 
                    && (profSegments[segId-1][1] == profSegments[segId][1]) && (profSegments[segId-1][4] != 0)) {
                    return true;
                } else {
                    return false;
                }
            }

            bool checkRegionExit(int segId) {
                // TODO: searching algorithm can be optimized
                int lineStart = profSegments[segId][0];
                int colStart = profSegments[segId][1];
                for (int i = 0; i < numProfRegions; i++) {
                    // skip gap region since they are always kept
                    if ((profRegions[i][7] != 3) && (lineStart == profRegions[i][2]) && (colStart == profRegions[i][3])) {
                        return true;
                    } else if (profRegions[i][0] > lineStart) {
                        return false;
                    }
                }
                return false;
            }

            bool checkBranchRegion(int segId) {
                // TODO: searching algorithm can be optimized
                int lineStart = profSegments[segId][0];
                int colStart = profSegments[segId][1];
                for (int i = 0; i < numProfBranches; i++) {
                    if (lineStart >= profBranches[i][0] && lineStart <= profBranches[i][2]) {
                        if (lineStart == profBranches[i][0] && colStart < profBranches[i][1]) {
                            continue;
                        } else if (lineStart == profBranches[i][2] && colStart > profBranches[i][3]) {
                            continue;
                        } else {
                            return true;
                        }
                    } else if (lineStart < profBranches[i][0]) {
                        return false;
                    }
                }
                return false;
            }

            bool checkKeep(int segId, std::string sourceBuf) {
                if (profSegments[segId][5] != 0) {
                    // gap region
                    return true;
                } else if (sourceBuf.find("if ") == 0) {
                    // starting with "if"
                    return true;
                } else if (checkBranchRegion(segId)) {
                    // within a branch region or following a branch region
                    return true;
                } else {
                    return false;
                }
            }

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

            double evalInputFunction(std::ifstream &profile, double th) {
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

            template<typename generator>
            double nf(double x, generator& g) {
                std::normal_distribution<double> d(0, 0.4);
                double step = d(g);
                // errs() << "step: " << step << "\n";
                double res = x + step;
                if (res < 0.0) {
                    res = 0.001;
                }
                return res;
            }

            double exploreInputFunction(std::ifstream &profile, const double init_th) {
                double th_old = init_th;
                int count = 100;
                // Simulated annealing algorithm
                double cost_old = evalInputFunction(profile, th_old);
                double th_best = th_old;
                double cost_best = cost_old;

                std::random_device rd;
                std::mt19937_64 g(rd());

                std::uniform_real_distribution<double> rf(0, 1);

                for (; count > 0; --count) {
                    double th_new = nf<decltype(g)>(th_old, g);
                    double cost_new = evalInputFunction(profile, th_new);

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
                errs() << "\n";

                return th_best;
            }

            int extractInputFunction(Function &F, BasicBlock *endBB) {
                std::ifstream srcFile(filename);
                std::ofstream outFile("inputSpec.cpp");
                // double threshold = 0.5;
                // double nt = threshold * avgExeCountPerInst;

                double threshold = exploreInputFunction(srcFile, 0.5);
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
                                program += insertExit(BBset, &(F.getEntryBlock()), BB, endBB, btID);
                                btID++;
                                lastRemoved = true;
                                copyEn = false;
                            } else {
                                // errs() << "keep\n";
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
                program += insertExit(BBset, &(F.getEntryBlock()), endBB, endBB, 0);

                program += "}\n";

                std::string prologue = generateInputMain(btID);
                program = prologue + program;

                outFile << program;

                return btID;
            }

            void generateHeader(int REGWIDTH, int numExit) {
                std::ofstream outFile("tmp.h");
                std::string program;
                for (auto varType = varTypeMap.begin(); varType != varTypeMap.end(); varType++) {
                    program += "typedef struct {\n";
                    auto varTypeName = varType->first;
                    int totalSize = 0;
                    std::string setFunction;
                    for (auto field = varType->second->begin(); field != varType->second->end(); field++) {
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
                    for (auto field = varType->second->rbegin(); field != varType->second->rend(); field++) {
                        tmp += (field->first + ", ");
                    }
                    program += tmp.substr(0, tmp.length()-2);
                    program += ");\n        return val;\n    }\n} ";
                    program += (varTypeName + ";\n\n");
                }
                outFile << program;
                outFile << numExit << "\n";
            }

            BasicBlock* getInputDoneBB(Function &F) {
                for (auto bb_it = F.begin(); bb_it != F.end(); bb_it++) {
                    for (auto inst_it = bb_it->begin(); inst_it != bb_it->end(); inst_it++) {
                        Instruction *inst = &*inst_it;
                        if (isa<CallInst>(*inst)) {
                            auto *tmp = dyn_cast<llvm::CallInst>(&*inst);
                            Function* foo = tmp->getCalledFunction();
                            if (foo->getName().contains("Input_done")) {
                                if (DILocation *Loc = inst->getDebugLoc()) {
                                    inputDoneLine = Loc->getLine();
                                }
                                return &*bb_it;
                            }
                        }
                    }
                }
                return NULL;
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

            BasicBlock* inferInputFunctions(Function &F) {
                std::vector<std::pair<BasicBlock*, int>> workstack;
                std::set<Value*> visited; // visited and contains Input_head

                BasicBlock *bb = &F.getEntryBlock();
                int index = getInputLengthBB(*bb, 0);
                workstack.emplace_back(bb, index);
                if (index != -1) {
                    visited.insert(bb);
                }

                BasicBlock *endBB;
                if (!(endBB = getInputDoneBB(F))) {
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
            
            virtual bool runOnModule(Module &M){
                initializeProfData();
                Function *main_func = initializeExeCount(M);
                initializeBBLoc(*main_func);
                initializeVarNames(*main_func);
                BasicBlock *endBB = inferInputFunctions(*main_func);
                livenessAnalysis(*main_func);
                int numExit = extractInputFunction(*main_func, endBB);
                generateHeader(192, numExit);

                delete [] numInstPerSeg;
                for (int i = 0; i < numProfSegments; i++) {
                    delete [] profSegments[i];
                }
                delete [] profSegments;
                for (int i = 0; i < numProfRegions; i++) {
                    delete [] profRegions[i];
                }
                delete [] profRegions;
                for (int i = 0; i < numProfBranches; i++) {
                    delete [] profBranches[i];
                }
                delete [] profBranches;
                primateCFG.close();
                interconnectCFG.close();
                primateHeader.close();
                assemblerHeader.close();
                return false;
            }

            virtual void getAnalysisUsage(AnalysisUsage &AU) const {
                AU.setPreservesAll();
            }

    };

    char PrimateBFUGen::ID = 0;

    static RegisterPass<PrimateBFUGen> X("primate", "primate pass");

    // static RegisterStandardPasses Y(
    // PassManagerBuilder::EP_EarlyAsPossible,
    // [](const PassManagerBuilder &Builder,
    //    legacy::PassManagerBase &PM) { PM.add(new PrimateBFUGen()); });

}

