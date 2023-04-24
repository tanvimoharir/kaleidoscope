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
