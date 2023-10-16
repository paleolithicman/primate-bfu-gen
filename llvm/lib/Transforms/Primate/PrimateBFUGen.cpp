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

#include "dataflow.h"
#include "inputGen.h"
#include "outputGen.h"

using namespace llvm;

namespace {

    class PrimateBFUGen : public ModulePass, public DataFlow<BitVector>, public inputGen, public outputGen, public AssemblyAnnotationWriter{

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

            std::vector<Value*> domain;
            std::vector<Value*> *bvIndexToInstrArg;                 //map values to their bitvector
            ValueMap<Value*, int> *valueToBitVectorIndex;           //map values (args and variables) to their bit vector index
            ValueMap<const Instruction*, BitVector*> *instrInSet;     //IN set for an instruction inside basic block
            ValueMap<Value*, Value*> *aliasMap;
            ValueMap<Value*, int> *branchLevel;
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
                }
                for (int i = 0; i < a.getNumElements(); i++) {
                    if (isa<llvm::IntegerType>(*elem)) {
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
                    if (isa<llvm::IntegerType>(**elem)) {
                        unsigned elemWidth = (*elem)->getIntegerBitWidth();
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

            virtual BitVector* getLiveoutVariables(std::set<BasicBlock*> &BBset, BasicBlock &endBB) {
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

            virtual BitVector* getLiveinVariables(std::set<BasicBlock*> &BBset) {
                BitVector* referred = new BitVector(domainSize, false);
                for (auto bb_it = BBset.begin(); bb_it != BBset.end(); bb_it++) {
                    BitVector *liveins = (*in)[*bb_it];
                    BitVector *liveouts = (*out)[*bb_it];
                    (*liveins) &= ((*liveouts).flip());
                    (*referred) |= (*liveins);
                }
                return referred;
            }

            virtual void initializeLiveinVars(BitVector* referred) {
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

            virtual int getRegIdx(Value* var) {
                Value *rootVar = (*aliasMap)[var];
                if (varRegMap.find(rootVar) != varRegMap.end()) {
                    return varRegMap[rootVar];
                } else {
                    errs() << "Error: Can't get the register index of the variable.\n";
                    return 0;
                }
            }

            virtual std::string insertInputExit(std::set<BasicBlock*> &BBset, BasicBlock* startBB, BasicBlock* endBB, BasicBlock* doneBB, int btID) {
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
            
            virtual bool runOnModule(Module &M){
                initializeProfData();
                Function *main_func = initializeExeCount(M);
                initializeBBLoc(*main_func);
                initializeVarNames(*main_func);
                livenessAnalysis(*main_func);

                BasicBlock *startBB, *endBB;
                // inputGen::inferInputFunctions(*main_func, startBB, endBB);
                // extractInputFunction(*main_func, endBB);

                outputGen::inferOutputFunctions(*main_func, startBB, endBB);
                outputGen::extractOutputFunction(*main_func, endBB);

                delete bvIndexToInstrArg;
                delete valueToBitVectorIndex;
                delete instrInSet;
                delete aliasMap;
                delete branchLevel;
                delete pointerMap;
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

