// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2014-2022 NVIDIA Corporation. All rights reserved.


#pragma once

#include "NvFlowString.h"

struct NvFlowPreprocessor;

struct NvFlowPreprocessorRange
{
	NvFlowUint64 begin;
	NvFlowUint64 end;
};

enum NvFlowPreprocessorTokenType
{
	eNvFlowPreprocessorTokenType_unknown = 0,	// unclassified
	eNvFlowPreprocessorTokenType_whitespace,	// 
	eNvFlowPreprocessorTokenType_newline,		// \n
	eNvFlowPreprocessorTokenType_comment,		// // comment
	eNvFlowPreprocessorTokenType_name,			// alpha_1234
	eNvFlowPreprocessorTokenType_number,		// 1234
	eNvFlowPreprocessorTokenType_string,		// "string"
	eNvFlowPreprocessorTokenType_char,			// 's'

	eNvFlowPreprocessorTokenType_pound,			// #
	eNvFlowPreprocessorTokenType_comma,			// ,
	eNvFlowPreprocessorTokenType_period,		// .
	eNvFlowPreprocessorTokenType_semicolon,		// ;
	eNvFlowPreprocessorTokenType_colon,			// :
	eNvFlowPreprocessorTokenType_equals,		// =
	eNvFlowPreprocessorTokenType_asterisk,		// *

	eNvFlowPreprocessorTokenType_leftParenthesis,	// (
	eNvFlowPreprocessorTokenType_rightParenthesis,	// )
	eNvFlowPreprocessorTokenType_leftBracket,		// [
	eNvFlowPreprocessorTokenType_rightBracket,		// ]
	eNvFlowPreprocessorTokenType_leftCurlyBrace,	// {
	eNvFlowPreprocessorTokenType_rightCurlyBrace,	// }

	eNvFlowPreprocessorTokenType_lessThan,			// <
	eNvFlowPreprocessorTokenType_greaterThan,		// >

	eNvFlowPreprocessorTokenType_anyWhitespace,		// For delimiter usage, aligns with NvFlowPreprocessorTokenIsWhitespace()

	eNvFlowPreprocessorTokenType_count,
	eNvFlowPreprocessorTokenType_maxEnum = 0x7FFFFFFF
};

struct NvFlowPreprocessorToken
{
	NvFlowPreprocessorTokenType type;
	const char* str;
};

NV_FLOW_INLINE NvFlowBool32 NvFlowPreprocessorTokenIsWhitespace(const NvFlowPreprocessorToken token)
{
	return token.type == eNvFlowPreprocessorTokenType_whitespace || 
		token.type == eNvFlowPreprocessorTokenType_newline ||
		token.type == eNvFlowPreprocessorTokenType_comment;
}

NV_FLOW_INLINE void NvFlowPreprocessorSkipWhitespaceTokens(NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens)
{
	while ((*pTokenIdx) < numTokens && NvFlowPreprocessorTokenIsWhitespace(tokens[(*pTokenIdx)]))
	{
		(*pTokenIdx)++;
	}
}

enum NvFlowPreprocessorType
{
	eNvFlowPreprocessorType_constant = 0,			// name
	eNvFlowPreprocessorType_statement = 1,			// name arg0 arg1;
	eNvFlowPreprocessorType_function = 2,			// name(arg0, arg1, arg2)
	eNvFlowPreprocessorType_index = 3,				// name[arg0] or name[arg0]= arg1 arg2 arg3;
	eNvFlowPreprocessorType_attribute = 4,			// [name(arg0, arg1, arg2)]
	eNvFlowPreprocessorType_line = 5,				// #name arg0 \n
	eNvFlowPreprocessorType_body = 6,				// name <arg0, arg1> arg2 arg3(arg4, arg5) { arg6; arg7; }	
	eNvFlowPreprocessorType_templateInstance = 7,	// name<arg0, arg1>
	eNvFlowPreprocessorType_statementComma = 8,		// "name arg0," or "name arg0)"

	eNvFlowPreprocessorType_maxEnum = 0x7FFFFFFF
};

struct NvFlowPreprocessorConstant
{
	const char* name;
	const char* value;
};

struct NvFlowPreprocessorFunction
{
	const char* name;
	NvFlowPreprocessorType type;
	void* userdata;
	char*(*substitute)(NvFlowPreprocessor* ptr, void* userdata, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens);
	NvFlowBool32 allowRecursion;
};

enum NvFlowPreprocessorMode
{
	eNvFlowPreprocessorMode_default = 0,			// Input string evaluated and substitution evaluated, no recursion
	eNvFlowPreprocessorMode_singlePass = 1,			// Input string evaluated once, no substitution evaluation
	eNvFlowPreprocessorMode_disable_passthrough = 2,	// Do not passthrough strings

	eNvFlowPreprocessorMode_maxEnum = 0x7FFFFFFF
};

NvFlowPreprocessor* NvFlowPreprocessorCreate(NvFlowStringPool* pool);

void NvFlowPreprocessorDestroy(NvFlowPreprocessor* ptr);

void NvFlowPreprocessorReset(NvFlowPreprocessor* ptr);

void NvFlowPreprocessorSetMode(NvFlowPreprocessor* ptr, NvFlowPreprocessorMode mode);

NvFlowPreprocessorMode NvFlowPreprocessorGetMode(NvFlowPreprocessor* ptr);

NvFlowStringPool* NvFlowPreprocessorStringPool(NvFlowPreprocessor* ptr);

void NvFlowPreprocessorAddConstants(NvFlowPreprocessor* ptr, NvFlowUint64 numConstants, const NvFlowPreprocessorConstant* constants);

void NvFlowPreprocessorAddFunctions(NvFlowPreprocessor* ptr, NvFlowUint64 numFunctions, const NvFlowPreprocessorFunction* functions);

NvFlowPreprocessorRange NvFlowPreprocessorExtractTokensDelimitedN(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowUint64 numDelimiters, const NvFlowPreprocessorTokenType* delimiters);

NvFlowPreprocessorRange NvFlowPreprocessorExtractTokensDelimited(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType delimiter);

const char* NvFlowPreprocessorExtractDelimited(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType delimiter);

const char* NvFlowPreprocessorExtractDelimitedN(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowUint64 numDelimiters, const NvFlowPreprocessorTokenType* delimiters);

const char* NvFlowPreprocessorExtractDelimitedPreserve(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType delimiter);

const char* NvFlowPreprocessorExtractIfType(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType type);

const char* NvFlowPreprocessorConcatTokens(NvFlowPreprocessor* ptr, const NvFlowPreprocessorToken* tokens, NvFlowUint64 numTokens);

NvFlowBool32 NvFlowPreprocessorFindKeyInSource(NvFlowPreprocessor* ptr, const NvFlowPreprocessorToken* keyTokens, NvFlowUint64 keyTokenCount, const NvFlowPreprocessorToken* sourceTokens, NvFlowUint64 sourceTokenCount, NvFlowUint64* pSourceIndex);

char* NvFlowPreprocessorExecute(NvFlowPreprocessor* ptr, const char* input);

void NvFlowPreprocessorTokenize(NvFlowPreprocessor* ptr, const char* input, NvFlowUint64* pTotalTokens, NvFlowPreprocessorToken** pTokens);

enum NvFlowPreprocessorGlobalType
{
	eNvFlowPreprocessorGlobalType_unknown = 0,			// Unknown global type
	eNvFlowPreprocessorGlobalType_statement = 1,		// ConstantBuffer<Params> gParams;
	eNvFlowPreprocessorGlobalType_function = 2,			// returnType functionName(arg1, arg2, arg3) { [functionbody] }
	eNvFlowPreprocessorGlobalType_attribute = 3,		// [name(arg0, arg1, arg2)]
	eNvFlowPreprocessorGlobalType_line = 4,				// #define CONSTANT \n

	eNvFlowPreprocessorGlobalType_maxEnum = 0x7FFFFFFF
};

char* NvFlowPreprocessorExecuteGlobal(NvFlowPreprocessor* ptr, const char* input, void* userdata, char*(*substitute)(NvFlowPreprocessor* ptr, void* userdata, NvFlowPreprocessorGlobalType globalType, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens));