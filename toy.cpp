#include "include/KaleidoscopeJIT.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace llvm;
using namespace llvm::orc;

//--------------------------------
//Lexer
//--------------------------------

//The lexer returns tokens [0-255] if it is an unknown character, otherwise one of these things.

enum Token {
	tok_eof = -1,

	//commands
	tok_def = -2,
	tok_extern = -3,

	//primary
	tok_identifier = -4,
	tok_number = -5,

	//control
	tok_if = -6,
	tok_then = -7,
	tok_else = -8,
	tok_for = -9,
	tok_in = -10,

	//operators
	tok_binary = -11,
	tok_unary = -12
};

static std::string IdentifierStr; //filled in if tok_identifier
static double NumVal; //filled in if tok_number

//gettok - Return the next token from standard input.
static int gettok() {
	static int LastChar = ' ';

	//skip any whitespace.
	while(isspace(LastChar)) {
		LastChar = getchar();
	}
	if (isalpha(LastChar)) {
		IdentifierStr = LastChar;
		while (isalnum((LastChar = getchar())))
			IdentifierStr += LastChar;

		if (IdentifierStr == "def")
			return tok_def;
		if(IdentifierStr == "extern")
			return tok_extern;
		if(IdentifierStr == "if")
			return tok_if;
		if(IdentifierStr == "then")
			return tok_then;
		if(IdentifierStr == "else")
			return tok_else;
		if(IdentifierStr == "for")
			return tok_for;
		if(IdentifierStr == "in")
			return tok_in;
		if(IdentifierStr == "binary")
			return tok_binary;
		if(IdentifierStr == "unary")
			return tok_unary;
		return tok_identifier;
	}

	if (isdigit(LastChar) || LastChar  == '.') {
		std::string NumStr;
		do {
			NumStr += LastChar;
			LastChar = getchar();
		} while (isdigit(LastChar) || LastChar == '.');

		NumVal = strtod(NumStr.c_str(), nullptr);
		return tok_number;
	}

	if (LastChar == '#') {
	//comment until end of line
		do
			LastChar = getchar();
		while (LastChar != EOF && LastChar != '\n' && LastChar != '\r');

		if (LastChar != EOF)
			return gettok();
	}

	//check for end of file, dont eat EOF
	if (LastChar == EOF)
		return tok_eof;

	//Otherwise, just returnnthe character as its ascii value
	int ThisChar = LastChar;
	LastChar = getchar();
	return ThisChar;
}

//===----------------------------------
//Abstract Syntax Tree (aka Parse Tree)
//===----------------------------------

//namespace is similar to static for variable meaning
//classes defined under it are only available within this 
//trnaslation unit

namespace {

/// ExprAST - Base class for all expressions nodes
class ExprAST {
public:
	virtual ~ExprAST() = default;
	virtual Value *codegen() = 0;
};

///NumberExprAST  - Expression class for numeric literal like "1.0"
class NumberExprAST : public ExprAST {
	double Val;

public:
	NumberExprAST(double Val) : Val(Val) {}
	Value *codegen() override;
};

///VariableExprAST - Expression class for referencing a variable like "a"
class VariableExprAST : public ExprAST {
	std::string Name;

public:
	VariableExprAST(const std::string &Name) : Name(Name) {}
	Value *codegen() override;
};

/// BinaryExprAST - Expression class for a binary operator

class BinaryExprAST : public ExprAST {
	char Op;
	std::unique_ptr<ExprAST> LHS, RHS;
public:
	BinaryExprAST(char Op, std::unique_ptr<ExprAST> LHS, std::unique_ptr<ExprAST> RHS) : Op(Op), LHS(std::move(LHS)), RHS(std::move(RHS)) {}
	Value *codegen() override;
};

/// UnaryExprAST - Expression class for a unary operator
class UnaryExprAST : public ExprAST {
	char Opcode;
	std::unique_ptr<ExprAST> Operand;
public:
	UnaryExprAST(char Opcode, std::unique_ptr<ExprAST> Operand) : Opcode(Opcode), Operand(std::move(Operand)) {}

	Value *codegen() override;
};

/// CallExprAST - Expression class for function calls.
class CallExprAST : public ExprAST {
	std::string Callee;
	std::vector<std::unique_ptr<ExprAST>> Args;

public:
	CallExprAST(const std::string& Callee, std::vector<std::unique_ptr<ExprAST>> Args) : Callee(Callee), Args(std::move(Args)) {}
	Value *codegen() override;
};

class IfExprAST : public ExprAST {
	std::unique_ptr<ExprAST> Cond, Then, Else;

	public:
		IfExprAST(std::unique_ptr<ExprAST> Cond, std::unique_ptr<ExprAST> Then, std::unique_ptr<ExprAST> Else)
		: Cond(std::move(Cond)), Then(std::move(Then)), Else(std::move(Else)) {}
		Value *codegen() override;
};

class ForExprAST : public ExprAST {
	std::string VarName;
	std::unique_ptr<ExprAST> Start, End, Step, Body;
public:
	ForExprAST(const std::string &VarName, std::unique_ptr<ExprAST> Start, std::unique_ptr<ExprAST> End, std::unique_ptr<ExprAST> Step, std::unique_ptr<ExprAST> Body)
		: VarName(VarName), Start(std::move(Start)), End(std::move(End)), Step(std::move(Step)), Body(std::move(Body)) {}

	Value *codegen() override;
};

/// PrototypeAST - This class represents the prototype for a func,
/// which captures its name, and its argument names (thus implicitly the number
/// of arguments the func takes).
class PrototypeAST{
	std::string Name;
	std::vector<std::string> Args;
	bool IsOperator;
	unsigned Precedence; //Precedence if a binary op.

public:
	PrototypeAST(const std::string& Name, std::vector<std::string> Args, bool IsOperator = false, unsigned Prec = 0) : Name(Name), Args(std::move(Args)), IsOperator(IsOperator), Precedence(Prec) {}
	
	Function *codegen();
	const std::string& getName() const { return Name; }

	bool isUnaryOp() const { return IsOperator && Args.size() == 1; }
	bool isBinaryOp() const { return IsOperator && Args.size() == 2; }

	char getOperatorName() const {
		assert(isUnaryOp() || isBinaryOp());
		return Name[Name.size() - 1];
	}

	unsigned getBinaryPrecedence() const { return Precedence; }

};

/// FunctionAST - this class represents a function definition itself
class FunctionAST {
	std::unique_ptr<PrototypeAST> Proto;
	std::unique_ptr<ExprAST> Body;
public:
	FunctionAST(std::unique_ptr<PrototypeAST> Proto, std::unique_ptr<ExprAST> Body) : Proto(std::move(Proto)), Body(std::move(Body)) {}

	Function *codegen();
};

}// end anonymous namespace

//===-------------------------------------------------
// Parser
//===-------------------------------------------------

/// Curtok/getNextTok - Provide a simple token buffer. Curtok is the current
/// token the parser is looking at, getNextToken reads another token from the 
/// lexer and updates CurTok with its results
static int CurTok;
static int getNextToken() { return CurTok = gettok(); }

/// Binop Precedence - This holds the precedence for each binary operator that is defined
static std::map<char, int> BinopPrecedence;

/// GetTokPrecedence - Get the precedence of the pending binary operator token
static int GetTokPrecedence() {
	if(!isascii(CurTok))
		return -1;

	// Make sure its a declared binop
	int TokPrec = BinopPrecedence[CurTok];
	if (TokPrec <= 0)
		return -1;
	return TokPrec;
}

/// LogError* - These are little helper functions for error handling
std::unique_ptr<ExprAST> LogError(const char* Str) {
	fprintf(stderr, "Error: %s\n", Str);
	return nullptr;
}

std::unique_ptr<PrototypeAST> LogErrorP(const char* Str) {
	LogError(Str);
	return nullptr;
}

static std::unique_ptr<ExprAST> ParseExpression();

/// numberexpr ::= number
static std::unique_ptr<ExprAST> ParseNumberExpr() {
	auto Result = std::make_unique<NumberExprAST>(NumVal);
	getNextToken(); //consume the number
	return std::move(Result);
}

/// parenexpr ::= '(' expression ')'
static std::unique_ptr<ExprAST> ParseParenExpr() {
	getNextToken(); //eat
	auto V = ParseExpression();
	if (!V)
		return nullptr;

	if (CurTok != ')')
		return LogError("expected ')'");
	getNextToken(); //eat
	return V;
}

/// identifierexpr
/// ::= identifier
/// ::= ideintifier '(' expression* ')'
static std::unique_ptr<ExprAST> ParseIdentifierExpr() {
	std::string IdName = IdentifierStr;

	getNextToken(); //eat identifier

	if (CurTok != '(') //simple variable ref.
		return std::make_unique<VariableExprAST>(IdName);

	// Call.
	getNextToken(); //eat
	std::vector<std::unique_ptr<ExprAST>> Args;
	if (CurTok != ')') {
		while (true) {
			if(auto Arg = ParseExpression())
				Args.push_back(std::move(Arg));
			else
				return nullptr;

			if (CurTok == ')')
				break;

			if(CurTok != ',')
				return LogError("Expected ')' pr ',' in argument list");
			getNextToken();
		}
	}
	//Eat the ')'
	getNextToken();
	
	return std::make_unique<CallExprAST>(IdName, std::move(Args));
}

/// ifexpr ::= 'if' expression 'then' expression 'else' exprssion
static std::unique_ptr<ExprAST> ParseIfExpr() {
        getNextToken(); //eat the if

        //condition
        auto Cond = ParseExpression();
        if (!Cond)
                return nullptr;

        if (CurTok != tok_then)
                return LogError("expected then");
        getNextToken(); //eat the then

        auto Then = ParseExpression();
        if (!Then)
                return nullptr;

        if (CurTok != tok_else)
                return LogError("expected else");
        getNextToken();

        auto Else = ParseExpression();
        if (!Else)
                return nullptr;

        return std::make_unique<IfExprAST>(std::move(Cond), std::move(Then), std::move(Else));

}

//forexpr ::= 'for' identifier '=' expr ',' expr(',' expr)? 'in' expression
static std::unique_ptr<ExprAST> ParseForExpr() {
	getNextToken(); //eat the for

	if (CurTok != tok_identifier)
		return LogError("expected identifier after for");

	std::string IdName = IdentifierStr;
	getNextToken(); 

	if (CurTok != '=')
		return LogError("expected '=' after for identfr");
	getNextToken();

	auto Start = ParseExpression();
	if (!Start)
		return nullptr;
	if (CurTok != ',')
		return LogError("expected ',' after for start value");
	getNextToken();
	auto End = ParseExpression();
	if (!End)
		return nullptr;

	// The step value is optional
	std::unique_ptr<ExprAST> Step;
	if (CurTok == ',') {
		getNextToken();
		Step = ParseExpression();
		if (!Step)
			return nullptr;
	}

	if (CurTok != tok_in)
		return LogError("expected 'in' after for");
	getNextToken();

	auto Body = ParseExpression();
	if (!Body)
		return nullptr;

	return std::make_unique<ForExprAST> (IdName, std::move(Start), std::move(End), std::move(Step), std::move(Body));

}

/// primary
/// ::= identifeirexpr
/// ::= numberexpr
/// ::= parenexpr
/// ::= ifexpr
/// ::= forexpr
static std::unique_ptr<ExprAST> ParsePrimary() {
	switch(CurTok) {
	default:
		return LogError("unknown token when expecting an expression");
	case tok_identifier:
		return ParseIdentifierExpr();
	case tok_number:
		return ParseNumberExpr();
	case '(':
		return ParseParenExpr();
	case tok_if:
		return ParseIfExpr();
	case tok_for:
		return ParseForExpr();
	}
}

static std::unique_ptr<ExprAST> ParseUnary() {
	// IF the current token is not an operator, it must be a primay expr
	if (!isascii(CurTok) || CurTok == '(' || CurTok == ',')
		return ParsePrimary();
	int Opc = CurTok;
	getNextToken();
	if (auto Operand = ParseUnary())
		return std::make_unique<UnaryExprAST>(Opc, std::move(Operand));
	return nullptr;
}

/// binorphs
///  ::= ('+' primary)*
static std::unique_ptr<ExprAST> ParseBinOpRHS(int ExprPrec, std::unique_ptr<ExprAST> LHS) {
	// If this is a binop, find its precedence
	while(true){
		int TokPrec = GetTokPrecedence();

		// If this is a binop that binds at least as tightly as the current binop,
		// consume it, otherwise we are done.
		if (TokPrec < ExprPrec)
			return LHS;
		// okay, we know this is a binop
		int BinOp = CurTok;
		getNextToken(); //eat binop

		// Parse the unary expression after the binary operator.
		auto RHS = ParseUnary();
		if(!RHS)
			return nullptr;

		//if binop binds less tightly with RHS than theoperator after RHS,
		// the pending operator take RHS as its LHS.
		int NextPrec = GetTokPrecedence();
		if(TokPrec < NextPrec) {
			RHS = ParseBinOpRHS(TokPrec + 1, std::move(RHS));
			if (!RHS)
				return nullptr;
		}

		// Merge LHS RHS
		LHS = std::make_unique<BinaryExprAST>(BinOp, std::move(LHS), std::move(RHS));
	}
}

/// expr
/// ::= primary binorphs
static std::unique_ptr<ExprAST> ParseExpression() {
	auto LHS = ParseUnary();
	if (!LHS)
		return nullptr;
	return ParseBinOpRHS(0, std::move(LHS));
}


/// prototype
/// ::= id '(' id* ')
static std::unique_ptr<PrototypeAST> ParsePrototype() {
	std::string FnName;
	
	unsigned Kind = 0;
	unsigned BinaryPrec = 30;

	switch (CurTok){
	default:
		return LogErrorP("Expected function name in prottype");
	case tok_identifier:
		FnName = IdentifierStr;
		Kind = 0;
		getNextToken();
		break;
	case tok_unary:
		getNextToken();
		if (!isascii(CurTok))
			return LogErrorP("Expected unary operator");
		FnName = "unary";
		FnName += (char)CurTok;
		Kind = 1;
		getNextToken();
		break;
	case tok_binary:
		getNextToken();
		if (!isascii(CurTok))
			return LogErrorP("Expected binary operator");
		FnName = "binary";
		FnName += (char)CurTok;
		Kind = 2;
		getNextToken();

		//Read the precedence if present.
		if (CurTok == tok_number) {
			if (NumVal < 1 || NumVal > 100)
				return LogErrorP("Invalid precedence: must be 1..100");
		BinaryPrec = (unsigned)NumVal;
		getNextToken();
		}
		break;
	}

	if (CurTok != '(')
		return LogErrorP("Expected '(' in prototype");

	std::vector<std::string> ArgNames;
	while (getNextToken() == tok_identifier)
		ArgNames.push_back(IdentifierStr);
	if(CurTok != ')')
		return LogErrorP("Expected ')' in prototype");
	//success
	getNextToken();

	// Verify right number of names for operator
	if (Kind && ArgNames.size() != Kind)
		return LogErrorP("Invalid number of operands for operator");

	return std::make_unique<PrototypeAST>(FnName, std::move(ArgNames), Kind != 0, BinaryPrec);
}

/// definition ::= 'def' prototype expression
static std::unique_ptr<FunctionAST> ParseDefinition() {
	getNextToken(); //eat next token
	auto Proto = ParsePrototype();
	if (!Proto)
		return nullptr;

	if(auto E = ParseExpression())
		return std::make_unique<FunctionAST>(std::move(Proto), std::move(E));
	return nullptr;
}

/// toplevelexpr ::=expression
static std::unique_ptr<FunctionAST> ParseTopLevelExpr() {
	if(auto E = ParseExpression()) {
		//make an anonymous proto
		auto Proto = std::make_unique<PrototypeAST>("__anon_expr", std::vector<std::string>());
		return std::make_unique<FunctionAST>(std::move(Proto), std::move(E));
	}
	return nullptr;
}

/// external ::= 'extern' prototype
static std::unique_ptr<PrototypeAST> ParseExtern() {
	getNextToken();
	return ParsePrototype();
}

//===-----------------------------------------
// Code generation
//===-----------------------------------------

static std::unique_ptr<LLVMContext> TheContext;
static std::unique_ptr<Module> TheModule;
static std::unique_ptr<IRBuilder<>> Builder;
static std::map<std::string, Value *> NamedValues;
static std::unique_ptr<legacy::FunctionPassManager> TheFPM;
static std::unique_ptr<KaleidoscopeJIT> TheJIT;
static std::map<std::string, std::unique_ptr<PrototypeAST>> FunctionProtos;
static ExitOnError ExitOnErr;

Value *LogErrorV(const char *Str) {
	LogError(Str);
	return nullptr;
}

Function *getFunction(std::string Name) {
	//First see if func has been added to current module
	if (auto *F = TheModule->getFunction(Name))
		return F;
	// if not check if we can codegen the declaration of prototype
	auto FI = FunctionProtos.find(Name);
	if (FI != FunctionProtos.end())
		return FI->second->codegen();
	//if not existing prototuype exist return null
	return nullptr;
}

Value *NumberExprAST::codegen() {
	return ConstantFP::get(*TheContext, APFloat(Val));
}

Value *VariableExprAST::codegen() {
	//look this variable up in the function
	Value *V = NamedValues[Name];
	if (!V)
		return LogErrorV("Unknown variable name");
	return V;
}

Value *BinaryExprAST::codegen() {
	Value *L = LHS->codegen();
	Value *R = RHS->codegen();

	if (!L || !R)
		return nullptr;

	switch (Op) {
	case '+':
		return Builder->CreateFAdd(L, R, "addtmp");
	case '-':
                return Builder->CreateFSub(L, R, "subtmp");
	case '*':
                return Builder->CreateFMul(L, R, "multmp");
	case '<':
                L = Builder->CreateFCmpULT(L, R, "cmptmp");
		//Convert bool 0/1 to double 0.0 or 1.0
		return Builder->CreateUIToFP(L, Type::getDoubleTy(*TheContext), "booltmp");
	default:
		break;
	}

	// if it wasnt builtin binary op it must be user defined one
	Function *F = getFunction(std::string("binary") + Op);

	assert(F && "binary operator not found!");

	Value *Ops[2] = { L, R };
	return Builder->CreateCall(F, Ops, "binop");

}

Value *UnaryExprAST::codegen() {
	Value *OperandV = Operand->codegen();
	if (!OperandV)
		return nullptr;

	Function *F = getFunction(std::string("unary") + Opcode);
	if (!F)
		return LogErrorV("Unknown unary operator");

	return Builder->CreateCall(F, OperandV, "unop");
}

Value *CallExprAST::codegen() {
	//Look up the name in the global module table
	Function *CalleeF = getFunction(Callee);
	if (!CalleeF)
		return LogErrorV("Unknown function referenced");
	//if argument mismatch error
	if (CalleeF->arg_size() != Args.size())
		return LogErrorV("Incorrect # arguments passed");
	std::vector<Value *> ArgsV;
	for(unsigned i = 0, e = Args.size(); i != e; ++i) {
		ArgsV.push_back(Args[i]->codegen());
		if (!ArgsV.back())
			return nullptr;
	}
	return Builder->CreateCall(CalleeF, ArgsV, "calltmp"); 
}

Value *IfExprAST::codegen() {
	Value *CondV = Cond->codegen();
	if (!CondV)
		return nullptr;

	//Convert condition to a bool by comparing non-equal to 0.0
	CondV = Builder->CreateFCmpONE(CondV, ConstantFP::get(*TheContext, APFloat(0.0)), "ifcond");

	Function *TheFunction = Builder->GetInsertBlock()->getParent();

	// Create blocks for the then anf else cases. Insert the 'then' block at the
	//end of the function
	BasicBlock *ThenBB = BasicBlock::Create(*TheContext, "then", TheFunction);
	BasicBlock *ElseBB = BasicBlock::Create(*TheContext, "else");
	BasicBlock *MergeBB = BasicBlock::Create(*TheContext, "ifcont");
	Builder->CreateCondBr(CondV, ThenBB, ElseBB);

	//Emit the value
	Builder->SetInsertPoint(ThenBB);

	Value *ThenV = Then->codegen();
	if (!ThenV)
		return nullptr;

	Builder->CreateBr(MergeBB);
	//Codegen of 'Then' can change the current block, update ThenBB for the PHI.
	ThenBB = Builder->GetInsertBlock();

	// Emit else block
	TheFunction->insert(TheFunction->end(), ElseBB);
	Builder->SetInsertPoint(ElseBB);

	Value *ElseV = Else->codegen();
	if (!ElseV)
		return nullptr;

	Builder->CreateBr(MergeBB);
	// codegen of Else can change the current block, update ElseBB for the PHI.
	ElseBB = Builder->GetInsertBlock();

	// Emit merge block
	TheFunction->insert(TheFunction->end(), MergeBB);
	Builder->SetInsertPoint(MergeBB);

	PHINode* PN = Builder->CreatePHI(Type::getDoubleTy(*TheContext), 2, "iftmp");
	PN->addIncoming(ThenV, ThenBB);
	PN->addIncoming(ElseV, ElseBB);
	return PN;
}

Value *ForExprAST::codegen() {
	//Emit the start code first, without 'variable' in scope
	Value *StartVal = Start->codegen();
	if (!StartVal)
		return nullptr;

	// Make the new basic block for the loop header, inserting after current block
	Function *TheFunction = Builder->GetInsertBlock()->getParent();
	BasicBlock *PreheaderBB = Builder->GetInsertBlock();
	BasicBlock *LoopBB = BasicBlock::Create(*TheContext, "loop", TheFunction);
	// Insert an explicit fall trhough from the current block to the LoopBB.
	Builder->CreateBr(LoopBB);
	//Start insertion in LoopBB.
	Builder->SetInsertPoint(LoopBB);

	//Start the PHI node with an entry for start.
	PHINode *Variable = Builder->CreatePHI(Type::getDoubleTy(*TheContext), 2, VarName);
	Variable->addIncoming(StartVal, PreheaderBB);

	Value *OldVal = NamedValues[VarName];
	NamedValues[VarName] = Variable;

	if (!Body->codegen())
		return nullptr;
	// Emit the step value
	Value *StepVal = nullptr;
	if (Step) {
		StepVal = Step->codegen();
		if (!StepVal)
			return nullptr;
	} else {
		StepVal = ConstantFP::get(*TheContext, APFloat(1.0));
	}
	Value *NextVar = Builder->CreateFAdd(Variable, StepVal, "nextvar");

	//compute end condition
	Value *EndCond = End->codegen();
	if (!EndCond)
		return nullptr;

	// Convert condition to a bool by comparing non-equal to 0.0.
	EndCond = Builder->CreateFCmpONE(EndCond, ConstantFP::get(*TheContext, APFloat(0.0)),"loopcond");
	//Create after the loop block and insert it
	BasicBlock *LoopEndBB = Builder->GetInsertBlock();
	BasicBlock *AfterBB = BasicBlock::Create(*TheContext, "afterloop", TheFunction);
	//Insert the conditional branch into the end of loopendbb
	Builder->CreateCondBr(EndCond, LoopBB, AfterBB);

	//Any new code will be inserted in AfterBB.
	Builder->SetInsertPoint(AfterBB);

	//Add a new entry to the PHI node for the backedge
	Variable->addIncoming(NextVar, LoopEndBB);

	//Restore the unshadowed variable
	if (OldVal)
		NamedValues[VarName] = OldVal;
	else
		NamedValues.erase(VarName);

	//for expr always returns 0.0
	return Constant::getNullValue(Type::getDoubleTy(*TheContext));
}


Function *PrototypeAST::codegen() {
	//Make the function type: double(double, double) etc.
	std::vector<Type *> Doubles(Args.size(), Type::getDoubleTy(*TheContext));
	FunctionType *FT = FunctionType::get(Type::getDoubleTy(*TheContext), Doubles, false);
	Function *F = Function::Create(FT, Function::ExternalLinkage, Name, TheModule.get());

	//Set names for all arguments
	unsigned Idx = 0;
	for (auto &Arg : F->args())
		Arg.setName(Args[Idx++]);
	return F;
}


Function *FunctionAST::codegen() {
	//Transfer ownership of prototype to funcprtos map,
	// but keep reference for use
	auto &P = *Proto;
	FunctionProtos[Proto->getName()] = std::move(Proto);
	Function *TheFunction = getFunction(P.getName());

	if (!TheFunction)
		return nullptr;
	//If this is an operator, install it
	if (P.isBinaryOp())
		BinopPrecedence[P.getOperatorName()] = P.getBinaryPrecedence();

	//Create a new basic block to start insertion into
	BasicBlock *BB = BasicBlock::Create(*TheContext, "entry", TheFunction);
	Builder->SetInsertPoint(BB);

	//Record the function arguments in the NamedValues map.
	NamedValues.clear();
	for(auto &Arg : TheFunction->args())
		NamedValues[std::string(Arg.getName())] = &Arg;

	if (Value *RetVal = Body->codegen()) {
	//Finish off the function
	Builder->CreateRet(RetVal);

	//Validate the generated code, checking for consistency.
	verifyFunction(*TheFunction);

	// Optimize the function
	TheFPM->run(*TheFunction);

	return TheFunction;
	}

	//Error reading body, remove function.
	TheFunction->eraseFromParent();
	return nullptr;
}


//===--------------------------------------------------
// Top level parsing and JIT Driver
//===--------------------------------------------------

static void InitializeModuleAndPassManager() {
	//Open a new context and module.
	TheContext = std::make_unique<LLVMContext>();
	TheModule = std::make_unique<Module>("my cool jit", *TheContext);
	TheModule->setDataLayout(TheJIT->getDataLayout());	

	// Create a new builder for the module.
	Builder = std::make_unique<IRBuilder<>>(*TheContext);

	// Create a new pass manager attached to it
	TheFPM = std::make_unique<legacy::FunctionPassManager>(TheModule.get());

	//do simple peephole optimizations and bit tweedling optiond
	TheFPM->add(createInstructionCombiningPass());

	// Reassociate expressions
	TheFPM->add(createReassociatePass());

	// Eliminate common subexpressions
	TheFPM->add(createGVNPass());

	// Simplify the control flow graph (deleteing unreachable nodes
	TheFPM->add(createCFGSimplificationPass());

	TheFPM->doInitialization();
}
	


static void HandleDefinition() {
	if (auto FnAST = ParseDefinition()) {
		if (auto *FnIR = FnAST->codegen()) {
			fprintf(stderr, "Read function definition:");
			FnIR->print(errs());
			fprintf(stderr, "\n");
			ExitOnErr(TheJIT->addModule(ThreadSafeModule(std::move(TheModule), std::move(TheContext))));
			InitializeModuleAndPassManager();
		}
	} else {
	//skip token for error recover
		getNextToken();
	}
}

static void HandleExtern() {
	if (auto ProtoAST = ParseExtern()) {
		if (auto *FnIR = ProtoAST->codegen()) {
			fprintf(stderr, "Read extern:");
			FnIR->print(errs());
			fprintf(stderr, "\n");
			FunctionProtos[ProtoAST->getName()] = std::move(ProtoAST);
		}
	} else {
		getNextToken(); //skip token for error reco
	}
}

static void HandleTopLevelExpr() {
	//Evaluate a top-level expr into an anonymous func.
	if (auto FnAST = ParseTopLevelExpr()) {
        	if (FnAST->codegen()) {
		//Create a resource tracker to track JIT'd memory allocated to our
		// anonyms expr -- that way we can free after executing.
		auto RT = TheJIT->getMainJITDylib().createResourceTracker();
		auto TSM = ThreadSafeModule(std::move(TheModule), std::move(TheContext));
		ExitOnErr(TheJIT->addModule(std::move(TSM), RT));
		InitializeModuleAndPassManager();

		// Search the JIT for the __anon__expr symbol.
		auto ExprSymbol = ExitOnErr(TheJIT->lookup("__anon_expr"));
		assert(ExprSymbol && "Function not found");

		// Get the symbol's address and cast it to the right type (takes no
		// arguments, returns a double so we can call itas a native function.
		double (*FP)() = (double (*)())(intptr_t)ExprSymbol.getAddress();
		fprintf(stderr, "Evaluated to %f\n", FP());

		//Delete the anonymous expr module from the JIT.
		ExitOnErr(RT->remove());
		}
	} else {
		getNextToken();
	}
}

/// top ::= definition | external | expression | ';'
static void MainLoop() {
	while(true) {
		fprintf(stderr, "ready> ");
		switch(CurTok) {
		case tok_eof:
			return;
		case ';':
			getNextToken();
			break;
		case tok_def:
			HandleDefinition();
			break;
		case tok_extern:
			HandleExtern();
			break;
		default:
			HandleTopLevelExpr();
			break;
		}
	}
}

//===----------------------------------------------------
// Library functions that can be externed from user code
//===----------------------------------------------------

#ifdef _WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif

///putchard - putchar that takes a double and returns 0
extern "C" DLLEXPORT double putchard(double X) {
	fputc((char)X, stderr);
	return 0;
}

///printd - printf that takes double and prints it as "%f\n", returning 0.
extern "C" DLLEXPORT double printd(double X) {
	fprintf(stderr, "%f\n", X);
	return 0;
}


//===-------------------------------
// Main driver code
//===-------------------------------

int main() {
	InitializeNativeTarget();
	InitializeNativeTargetAsmPrinter();
	InitializeNativeTargetAsmParser();

	//Install standard binary operators
	//1 is the lowest precedence
	BinopPrecedence['<'] = 10;
	BinopPrecedence['+'] = 20;
	BinopPrecedence['-'] = 20;
	BinopPrecedence['*'] = 40;

	//Prime the first token
	fprintf(stderr, "ready> ");
	getNextToken();

	TheJIT = ExitOnErr(KaleidoscopeJIT::Create());

	//Make the module which holds all the code.
	InitializeModuleAndPassManager();

	//Run the main "interpreter loop" now
	MainLoop();

	//Print out all of the generated code
	//TheModule->print(errs(), nullptr);
	return 0;
}
