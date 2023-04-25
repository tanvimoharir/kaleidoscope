#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
};

///NumberExprAST  - Expression class for numeric literal like "1.0"
class NumberExprAST : public ExprAST {
	double Val;

public:
	NumberExprAST(double Val) : Val(Val) {}
};

///VariableExprAST - Expression class for referencing a variable like "a"
class VariableExprAST : public ExprAST {
	std::string Name;

public:
	VariableExprAST(const std::string &Name) : Name(Name) {}
};

/// BinaryExprAST - Expression class for a binary operator

class BinaryExprAST : public ExprAST {
	char Op;
	std::unique_ptr<ExprAST> LHS, RHS;
public:
	BinaryExprAST(char Op, std::unique_ptr<ExprAST> LHS, std::unique_ptr<ExprAST> RHS) : Op(Op), LHS(std::move(LHS)), RHS(std::move(RHS)) {}
};

/// CallExprAST - Expression class for function calls.
class CallExprAST : public ExprAST {
	std::string Callee;
	std::vector<std::unique_ptr<ExprAST>> Args;

public:
	CallExprAST(const std::string& Callee, std::vector<std::unique_ptr<ExprAST>> Args) : Callee(Callee), Args(std::move(Args)) {}
};

/// PrototypeAST - This class represents the prototype for a func,
/// which captures its name, and its argument names (thus implicitly the number
/// of arguments the func takes).
class PrototypeAST{
	std::string Name;
	std::vector<std::string> Args;

public:
	PrototypeAST(const std::string& Name, std::vector<std::string> Args) : Name(Name), Args(std::move(Args)) {}

	const std::string& getName() const { return Name; }
};

/// FunctionAST - this class represents a function definition itself
class FunctionAST {
	std::unique_ptr<PrototypeAST> Proto;
	std::unique_ptr<ExprAST> Body;
public:
	FunctionAST(std::unique_ptr<PrototypeAST> Proto, std::unique_ptr<ExprAST> Body) : Proto(std::move(Proto)), Body(std::move(Body)) {}
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

/// primary
/// ::= identifeirexpr
/// ::= numberexpr
/// ::= parenexpr
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
	}
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

		// Parse the primary expression after the binary operator.
		auto RHS = ParsePrimary();
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
	auto LHS = ParsePrimary();
	if (!LHS)
		return nullptr;
	return ParseBinOpRHS(0, std::move(LHS));
}
