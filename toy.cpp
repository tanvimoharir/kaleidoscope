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
	tok_eof = -1;

	//commands
	tok_def = -2;
	tok_extern = -3;

	//primary
	tok_identifier = -4;
	tok_number = -5;
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
			IdentifierStr += LastChar

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
