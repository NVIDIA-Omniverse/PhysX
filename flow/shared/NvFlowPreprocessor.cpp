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


#include "NvFlowPreprocessor.h"

#include "NvFlowArray.h"

#include <stdlib.h>
#include <stdio.h>

struct NvFlowPreprocessorItem
{
	NvFlowPreprocessorFunction function;
	NvFlowPreprocessorToken* tokens_data;
	NvFlowUint64 tokens_size;
	NvFlowBool32 isEnabled;
};

struct NvFlowPreprocessor
{
	NvFlowStringPool* stringPool = nullptr;
	int currentLevel = 0;

	NvFlowPreprocessorMode mode = eNvFlowPreprocessorMode_default;

	NvFlowArray<NvFlowPreprocessorItem, 16u> items;

	NvFlowArray<const char*> stringStack;

	NvFlowArray<const char*, 8u> tempStrViews;
	NvFlowArray<NvFlowPreprocessorToken> tempTokens;
};

NvFlowPreprocessor* NvFlowPreprocessorCreate(NvFlowStringPool* pool)
{
	auto ptr = new NvFlowPreprocessor();

	ptr->stringPool = pool;
	ptr->currentLevel = 0;

	return ptr;
}

void NvFlowPreprocessorDestroy(NvFlowPreprocessor* ptr)
{
	delete ptr;
}

void NvFlowPreprocessorReset(NvFlowPreprocessor* ptr)
{
	ptr->currentLevel = 0;
	ptr->mode = eNvFlowPreprocessorMode_default;

	ptr->items.size = 0u;

	ptr->stringStack.size = 0u;
	ptr->tempStrViews.size = 0u;
	ptr->tempTokens.size = 0u;
}

void NvFlowPreprocessorSetMode(NvFlowPreprocessor* ptr, NvFlowPreprocessorMode mode)
{
	ptr->mode = mode;
}

NvFlowPreprocessorMode NvFlowPreprocessorGetMode(NvFlowPreprocessor* ptr)
{
	return ptr->mode;
}

NvFlowStringPool* NvFlowPreprocessorStringPool(NvFlowPreprocessor* ptr)
{
	return ptr->stringPool;
}

void NvFlowPreprocessor_addItem(NvFlowPreprocessor* ptr, const NvFlowPreprocessorFunction* pFunction)
{
	NvFlowPreprocessorItem item = {};
	item.function = *pFunction;
	item.isEnabled = NV_FLOW_TRUE;

	const char* tokenStr = item.function.name;
	if (item.function.type == eNvFlowPreprocessorType_function)
	{
		tokenStr = NvFlowStringConcat(ptr->stringPool, tokenStr, "(");
	}
	else if (item.function.type == eNvFlowPreprocessorType_index)
	{
		tokenStr = NvFlowStringConcat(ptr->stringPool, tokenStr, "[");
	}
	else if (item.function.type == eNvFlowPreprocessorType_attribute)
	{
		tokenStr = NvFlowStringConcat(ptr->stringPool, "[", tokenStr);
	}
	else if (item.function.type == eNvFlowPreprocessorType_line)
	{
		tokenStr = NvFlowStringConcat(ptr->stringPool, "#", tokenStr);
	}
	else if (item.function.type == eNvFlowPreprocessorType_templateInstance)
	{
		tokenStr = NvFlowStringConcat(ptr->stringPool, tokenStr, "<");
	}
	else
	{
		tokenStr = NvFlowStringDup(ptr->stringPool, tokenStr);
	}

	NvFlowPreprocessorTokenize(ptr, tokenStr, &item.tokens_size, &item.tokens_data);

	ptr->items.pushBack(item);
}

char* NvFlowPreprocessor_substituteConstant(NvFlowPreprocessor* ptr, void* userdata, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens)
{
	const char* value = (const char*)userdata;
	return NvFlowStringDup(ptr->stringPool, value);
}

void NvFlowPreprocessorAddConstants(NvFlowPreprocessor* ptr, NvFlowUint64 numConstants, const NvFlowPreprocessorConstant* constants)
{
	for (NvFlowUint64 idx = 0u; idx < numConstants; idx++)
	{
		char* valueStr = NvFlowStringDup(ptr->stringPool, constants[idx].value);

		NvFlowPreprocessorFunction function = {};
		function.name = constants[idx].name;
		function.type = eNvFlowPreprocessorType_constant;
		function.userdata = valueStr;
		function.substitute = NvFlowPreprocessor_substituteConstant;

		NvFlowPreprocessor_addItem(ptr, &function);
	}
}

void NvFlowPreprocessorAddFunctions(NvFlowPreprocessor* ptr, NvFlowUint64 numFunctions, const NvFlowPreprocessorFunction* functions)
{
	for (NvFlowUint64 idx = 0u; idx < numFunctions; idx++)
	{
		NvFlowPreprocessor_addItem(ptr, &functions[idx]);
	}
}

char NvFlowPreprocessor_peekChar(const char* input, NvFlowUint64 inputIdx, NvFlowUint64 input_size)
{
	char ret = '\0';
	if (inputIdx < input_size)
	{
		ret = input[inputIdx];
	}
	return ret;
}

NvFlowBool32 NvFlowPreprocessor_whitespaceButNotNewline(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	if (NvFlowCharIsWhiteSpaceButNotNewline(c0))
	{
		NvFlowUint64 beginIdx = inputIdx;
		inputIdx++;
		for (; inputIdx < input_size; inputIdx++)
		{
			c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);

			if (!NvFlowCharIsWhiteSpaceButNotNewline(c0))
			{
				break;
			}
		}
		*pOutput_size = inputIdx - beginIdx;
		*pOutput = input + beginIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_continuation(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	char c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);
	if (c0 == '\\' && c1 == '\n')
	{
		*pOutput_size = 2;
		*pOutput = input + inputIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_whitespace(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	if (NvFlowPreprocessor_whitespaceButNotNewline(pOutput, pOutput_size, input, input_size, inputIdx))
	{
		return NV_FLOW_TRUE;
	}
	return NvFlowPreprocessor_continuation(pOutput, pOutput_size, input, input_size, inputIdx);
}

NvFlowBool32 NvFlowPreprocessor_newline(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	if (c0 == '\n')
	{
		*pOutput_size = 1;
		*pOutput = input + inputIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_commentLine(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	char c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

	if (c0 == '/' && c1 == '/')
	{
		NvFlowUint64 beginIdx = inputIdx;
		inputIdx++;
		for (; inputIdx < input_size; inputIdx++)
		{
			c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
			c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

			if (c0 == '\n')
			{
				break;
			}
		}
		*pOutput_size = inputIdx - beginIdx;
		*pOutput = input + beginIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_commentMultiLine(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	char c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

	if (c0 == '/' && c1 == '*')
	{
		NvFlowUint64 beginIdx = inputIdx;
		inputIdx++;
		for (; inputIdx < input_size; inputIdx++)
		{
			c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
			c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

			if (c0 == '*' && c1 == '/')
			{
				inputIdx += 2;
				break;
			}
		}
		*pOutput_size = inputIdx - beginIdx;
		*pOutput = input + beginIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_comment(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	if (NvFlowPreprocessor_commentLine(pOutput, pOutput_size, input, input_size, inputIdx))
	{
		return NV_FLOW_TRUE;
	}
	return NvFlowPreprocessor_commentMultiLine(pOutput, pOutput_size, input, input_size, inputIdx);
}

NvFlowBool32 NvFlowPreprocessor_name(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	if (NvFlowCharIsAlphaUnderscore(c0))
	{
		NvFlowUint64 beginIdx = inputIdx;
		inputIdx++;
		for (; inputIdx < input_size; inputIdx++)
		{
			c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);

			if (!NvFlowCharIsAlphaNum(c0))
			{
				break;
			}
		}
		*pOutput_size = inputIdx - beginIdx;
		*pOutput = input + beginIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_number(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	if (NvFlowCharIsNum(c0))
	{
		NvFlowUint64 beginIdx = inputIdx;
		inputIdx++;
		for (; inputIdx < input_size; inputIdx++)
		{
			c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);

			if (!(NvFlowCharIsAlphaNum(c0) || (c0 == '.')))
			{
				break;
			}
		}
		*pOutput_size = inputIdx - beginIdx;
		*pOutput = input + beginIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_string(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	char c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

	if (c0 == '\"')
	{
		NvFlowUint64 beginIdx = inputIdx;
		inputIdx++;
		for (; inputIdx < input_size; inputIdx++)
		{
			c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
			c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

			if (c0 == '\"')
			{
				inputIdx++;
				break;
			}
			else if (c0 == '\\' && c1 == '\"')
			{
				inputIdx++;
			}
		}
		*pOutput_size = inputIdx - beginIdx;
		*pOutput = input + beginIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_char(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	char c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

	if (c0 == '\'')
	{
		NvFlowUint64 beginIdx = inputIdx;
		inputIdx++;
		for (; inputIdx < input_size; inputIdx++)
		{
			c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
			c1 = NvFlowPreprocessor_peekChar(input, inputIdx + 1, input_size);

			if (c0 == '\'')
			{
				inputIdx++;
				break;
			}
			else if (c0 == '\\' && c1 == '\'')
			{
				inputIdx++;
			}
		}
		*pOutput_size = inputIdx - beginIdx;
		*pOutput = input + beginIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

NvFlowBool32 NvFlowPreprocessor_specialChar(const char** pOutput, NvFlowUint64* pOutput_size, const char* input, NvFlowUint64 input_size, NvFlowUint64 inputIdx, char specialChar)
{
	char c0 = NvFlowPreprocessor_peekChar(input, inputIdx + 0, input_size);
	if (c0 == specialChar)
	{
		*pOutput_size = 1;
		*pOutput = input + inputIdx;
		return NV_FLOW_TRUE;
	}
	return NV_FLOW_FALSE;
}

void NvFlowPreprocessorTokenize(NvFlowPreprocessor* ptr, const char* input, NvFlowUint64* pTotalTokens, NvFlowPreprocessorToken** pTokens)
{
	NvFlowStringPool* pool = ptr->stringPool;

	ptr->tempTokens.size = 0u;

	NvFlowUint64 input_size = NvFlowStringLength(input);
	NvFlowUint64 inputIdx = 0u;
	while (inputIdx < input_size)
	{
		char c0 = input[inputIdx];
		char c1 = '\0';
		if (inputIdx + 1 < input_size)
		{
			c1 = input[inputIdx + 1u];
		}

		// default to single char token
		NvFlowPreprocessorToken token = { eNvFlowPreprocessorTokenType_unknown, "InvalidToken" };

		NvFlowUint64 output_size = 1;
		const char* output = input + inputIdx;

		if (NvFlowPreprocessor_whitespace(&output, &output_size, input, input_size, inputIdx))
		{
			token.type = eNvFlowPreprocessorTokenType_whitespace;
		}
		else if (NvFlowPreprocessor_newline(&output, &output_size, input, input_size, inputIdx))
		{
			token.type = eNvFlowPreprocessorTokenType_newline;
		}
		else if (NvFlowPreprocessor_comment(&output, &output_size, input, input_size, inputIdx))
		{
			token.type = eNvFlowPreprocessorTokenType_comment;
		}
		else if (NvFlowPreprocessor_name(&output, &output_size, input, input_size, inputIdx))
		{
			token.type = eNvFlowPreprocessorTokenType_name;
		}
		else if (NvFlowPreprocessor_number(&output, &output_size, input, input_size, inputIdx))
		{
			token.type = eNvFlowPreprocessorTokenType_number;
		}
		else if (NvFlowPreprocessor_string(&output, &output_size, input, input_size, inputIdx))
		{
			token.type = eNvFlowPreprocessorTokenType_string;
		}
		else if (NvFlowPreprocessor_char(&output, &output_size, input, input_size, inputIdx))
		{
			token.type = eNvFlowPreprocessorTokenType_char;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '#'))
		{
			token.type = eNvFlowPreprocessorTokenType_pound;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, ','))
		{
			token.type = eNvFlowPreprocessorTokenType_comma;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '.'))
		{
			token.type = eNvFlowPreprocessorTokenType_period;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, ';'))
		{
			token.type = eNvFlowPreprocessorTokenType_semicolon;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, ':'))
		{
			token.type = eNvFlowPreprocessorTokenType_colon;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '='))
		{
			token.type = eNvFlowPreprocessorTokenType_equals;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '*'))
		{
			token.type = eNvFlowPreprocessorTokenType_asterisk;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '('))
		{
			token.type = eNvFlowPreprocessorTokenType_leftParenthesis;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, ')'))
		{
			token.type = eNvFlowPreprocessorTokenType_rightParenthesis;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '['))
		{
			token.type = eNvFlowPreprocessorTokenType_leftBracket;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, ']'))
		{
			token.type = eNvFlowPreprocessorTokenType_rightBracket;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '{'))
		{
			token.type = eNvFlowPreprocessorTokenType_leftCurlyBrace;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '}'))
		{
			token.type = eNvFlowPreprocessorTokenType_rightCurlyBrace;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '<'))
		{
			token.type = eNvFlowPreprocessorTokenType_lessThan;
		}
		else if (NvFlowPreprocessor_specialChar(&output, &output_size, input, input_size, inputIdx, '>'))
		{
			token.type = eNvFlowPreprocessorTokenType_greaterThan;
		}

		// duplicate output to null terminated string
		token.str = NvFlowStringFromView(pool, output, output_size);

		ptr->tempTokens.pushBack(token);

		// advance past token
		inputIdx += output_size;
	}

	auto tokenData = (NvFlowPreprocessorToken*)NvFlowStringPoolAllocate(pool, ptr->tempTokens.size * sizeof(NvFlowPreprocessorToken));
	for (NvFlowUint64 idx = 0u; idx < ptr->tempTokens.size; idx++)
	{
		tokenData[idx] = ptr->tempTokens[idx];
	}

	*pTokens = tokenData;
	*pTotalTokens = ptr->tempTokens.size;

	ptr->tempTokens.size = 0u;
}

NvFlowBool32 NvFlowPreprocessorFindKeyInSource(NvFlowPreprocessor* ptr, const NvFlowPreprocessorToken* keyTokens, NvFlowUint64 keyTokenCount, const NvFlowPreprocessorToken* sourceTokens, NvFlowUint64 sourceTokenCount, NvFlowUint64* pSourceIndex)
{
	NvFlowUint64 keyTokenIdx = 0u;
	NvFlowUint64 sourceTokenIdx = 0u;
	NvFlowUint64 matches = 0u;
	NvFlowUint64 keyTestCount = 0u;
	while (keyTokenIdx < keyTokenCount && sourceTokenIdx < sourceTokenCount)
	{
		NvFlowPreprocessorSkipWhitespaceTokens(&keyTokenIdx, keyTokenCount, keyTokens);
		NvFlowPreprocessorSkipWhitespaceTokens(&sourceTokenIdx, sourceTokenCount, sourceTokens);

		if (keyTokenIdx < keyTokenCount)
		{
			keyTestCount++;
		}
		if (keyTokenIdx < keyTokenCount && sourceTokenIdx < sourceTokenCount)
		{
			if (keyTokens[keyTokenIdx].type == sourceTokens[sourceTokenIdx].type)
			{
				if (keyTokens[keyTokenIdx].type == eNvFlowPreprocessorTokenType_name)
				{
					if (NvFlowStringCompare(keyTokens[keyTokenIdx].str, sourceTokens[sourceTokenIdx].str) == 0)
					{
						matches++;
					}
				}
				else
				{
					matches++;
				}
			}
		}

		keyTokenIdx++;
		sourceTokenIdx++;
	}
	if (pSourceIndex)
	{
		*pSourceIndex += sourceTokenIdx;
	}
	return (matches > 0 && matches == keyTestCount) ? NV_FLOW_TRUE : NV_FLOW_FALSE;
}

NvFlowPreprocessorRange NvFlowPreprocessorExtractTokensDelimitedN(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowUint64 numDelimiters, const NvFlowPreprocessorTokenType* delimiters)
{
	NvFlowUint64 beginTokenIdx = (*pTokenIdx);
	NvFlowPreprocessorRange range = { beginTokenIdx, beginTokenIdx };
	if (numDelimiters > 0u)
	{
		NvFlowUint64 localTokenIdx = beginTokenIdx;
		range = NvFlowPreprocessorExtractTokensDelimited(ptr, &localTokenIdx, numTokens, tokens, delimiters[0u]);
		(*pTokenIdx) = localTokenIdx;
	}
	for (NvFlowUint64 delimiterIdx = 1u; delimiterIdx < numDelimiters; delimiterIdx++)
	{
		NvFlowUint64 localTokenIdx = beginTokenIdx;
		NvFlowPreprocessorRange localRange = NvFlowPreprocessorExtractTokensDelimited(ptr, &localTokenIdx, numTokens, tokens, delimiters[delimiterIdx]);
		if (localRange.end < range.end)
		{
			range = localRange;
			(*pTokenIdx) = localTokenIdx;
		}
	}
	return range;
}

NvFlowPreprocessorRange NvFlowPreprocessorExtractTokensDelimited(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType delimiter)
{
	NvFlowPreprocessorRange range = { (*pTokenIdx), (*pTokenIdx) };

	NvFlowPreprocessorTokenType rightType = eNvFlowPreprocessorTokenType_rightParenthesis;
	NvFlowPreprocessorTokenType leftType = eNvFlowPreprocessorTokenType_leftParenthesis;
	if (delimiter == eNvFlowPreprocessorTokenType_greaterThan)
	{
		rightType = eNvFlowPreprocessorTokenType_greaterThan;
		leftType = eNvFlowPreprocessorTokenType_lessThan;
	}

	bool delimiterIsScopeEnd = (
		delimiter == eNvFlowPreprocessorTokenType_rightParenthesis ||
		delimiter == eNvFlowPreprocessorTokenType_rightBracket ||
		delimiter == eNvFlowPreprocessorTokenType_rightCurlyBrace ||
		delimiter == eNvFlowPreprocessorTokenType_greaterThan
		);
	int scopeIdx = delimiterIsScopeEnd ? 1 : 0;

	for (; (*pTokenIdx) < numTokens; (*pTokenIdx)++)
	{
		// scope end is 'before' the end symbol
		if (tokens[(*pTokenIdx)].type == eNvFlowPreprocessorTokenType_rightParenthesis ||
			tokens[(*pTokenIdx)].type == eNvFlowPreprocessorTokenType_rightBracket ||
			tokens[(*pTokenIdx)].type == eNvFlowPreprocessorTokenType_rightCurlyBrace ||
			tokens[(*pTokenIdx)].type == rightType)
		{
			scopeIdx--;
		}

		if (scopeIdx == 0 && tokens[(*pTokenIdx)].type == delimiter)
		{
			(*pTokenIdx)++;
			break;
		}
		else if (scopeIdx == 0 && delimiter == eNvFlowPreprocessorTokenType_anyWhitespace && NvFlowPreprocessorTokenIsWhitespace(tokens[(*pTokenIdx)]))
		{
			(*pTokenIdx)++;
			break;
		}
		else
		{
			range.end++;
		}

		// scope begin is 'after' the start symbol
		if (tokens[(*pTokenIdx)].type == eNvFlowPreprocessorTokenType_leftParenthesis ||
			tokens[(*pTokenIdx)].type == eNvFlowPreprocessorTokenType_leftBracket ||
			tokens[(*pTokenIdx)].type == eNvFlowPreprocessorTokenType_leftCurlyBrace ||
			tokens[(*pTokenIdx)].type == leftType)
		{
			scopeIdx++;
		}
	}

	return range;
}

const char* NvFlowPreprocessorExtractDelimitedN(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowUint64 numDelimiters, const NvFlowPreprocessorTokenType* delimiters)
{
	ptr->tempStrViews.size = 0u;

	NvFlowPreprocessorSkipWhitespaceTokens(pTokenIdx, numTokens, tokens);

	NvFlowPreprocessorRange range = NvFlowPreprocessorExtractTokensDelimitedN(ptr, pTokenIdx, numTokens, tokens, numDelimiters, delimiters);

	NvFlowPreprocessorToken prevPushedToken = {};
	for (NvFlowUint64 idx = range.begin; idx < range.end; idx++)
	{
		if (NvFlowPreprocessorTokenIsWhitespace(tokens[idx]))
		{
			continue;
		}
		else
		{
			if (tokens[idx].type == eNvFlowPreprocessorTokenType_name ||
				tokens[idx].type == eNvFlowPreprocessorTokenType_number)
			{
				if (prevPushedToken.type == eNvFlowPreprocessorTokenType_name ||
					prevPushedToken.type == eNvFlowPreprocessorTokenType_number)
				{
					ptr->tempStrViews.pushBack(" ");
				}
			}
			ptr->tempStrViews.pushBack(tokens[idx].str);
			prevPushedToken = tokens[idx];
		}
	}

	char* output = NvFlowStringConcatN(ptr->stringPool, ptr->tempStrViews.data, ptr->tempStrViews.size);
	ptr->tempStrViews.size = 0u;
	return output;
}

const char* NvFlowPreprocessorExtractDelimited(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType delimiter)
{
	return NvFlowPreprocessorExtractDelimitedN(ptr, pTokenIdx, numTokens, tokens, 1u, &delimiter);
}

const char* NvFlowPreprocessorExtractDelimitedPreserve(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType delimiter)
{
	NvFlowPreprocessorRange range = NvFlowPreprocessorExtractTokensDelimited(ptr, pTokenIdx, numTokens, tokens, delimiter);

	return NvFlowPreprocessorConcatTokens(ptr, tokens + range.begin, range.end - range.begin);
}

const char* NvFlowPreprocessorExtractIfType(NvFlowPreprocessor* ptr, NvFlowUint64* pTokenIdx, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens, NvFlowPreprocessorTokenType type)
{
	const char* ret = nullptr;

	NvFlowPreprocessorSkipWhitespaceTokens(pTokenIdx, numTokens, tokens);

	if ((*pTokenIdx) < numTokens && tokens[(*pTokenIdx)].type == type)
	{
		ret = tokens[(*pTokenIdx)].str;
		(*pTokenIdx)++;
	}
	return ret;
}

const char* NvFlowPreprocessorConcatTokens(NvFlowPreprocessor* ptr, const NvFlowPreprocessorToken* tokens, NvFlowUint64 numTokens)
{
	ptr->tempStrViews.size = 0u;

	for (NvFlowUint64 idx = 0u; idx < numTokens; idx++)
	{
		ptr->tempStrViews.pushBack(tokens[idx].str);
	}

	char* output = NvFlowStringConcatN(ptr->stringPool, ptr->tempStrViews.data, ptr->tempStrViews.size);
	ptr->tempStrViews.size = 0u;
	return output;
}

char* NvFlowPreprocessorExecute(NvFlowPreprocessor* ptr, const char* input)
{
	// increment level
	ptr->currentLevel++;

	NvFlowUint64 stringStackBegin = ptr->stringStack.size;

	// tokenize
	NvFlowPreprocessorToken* tokenStack_data = nullptr;
	NvFlowUint64 tokenStack_size = 0u;
	NvFlowPreprocessorTokenize(ptr, input, &tokenStack_size, &tokenStack_data);

	// process tokens
	for (NvFlowUint64 tokenIdx = 0u; tokenIdx < tokenStack_size; tokenIdx++)
	{
		NvFlowPreprocessorToken firstToken = tokenStack_data[tokenIdx];

		if (NvFlowPreprocessorTokenIsWhitespace(firstToken))
		{
			if (ptr->mode == eNvFlowPreprocessorMode_disable_passthrough)
			{
				// NOP
			}
			else
			{
				ptr->stringStack.pushBack(firstToken.str);
			}
		}
		else
		{
			NvFlowUint64 itemIdx = 0u;
			for (; itemIdx < ptr->items.size; itemIdx++)
			{
				const NvFlowPreprocessorItem item = ptr->items[itemIdx];
				NvFlowUint64 compareSourceIdx = tokenIdx;

				if (item.isEnabled && NvFlowPreprocessorFindKeyInSource(ptr,
					item.tokens_data, item.tokens_size,
					tokenStack_data + tokenIdx, tokenStack_size - tokenIdx,
					&compareSourceIdx))
				{
					NvFlowUint64 childTokenBegin = tokenIdx;
					NvFlowUint64 childTokenEnd = tokenIdx;
					if (item.function.type == eNvFlowPreprocessorType_constant)
					{
						childTokenEnd = compareSourceIdx;
					}
					else if (item.function.type == eNvFlowPreprocessorType_statement)
					{
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_semicolon);
					}
					else if (item.function.type == eNvFlowPreprocessorType_statementComma)
					{
						NvFlowPreprocessorTokenType delimiters[2u] = { eNvFlowPreprocessorTokenType_comma, eNvFlowPreprocessorTokenType_rightParenthesis };
						NvFlowPreprocessorExtractTokensDelimitedN(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, 2u, delimiters);
					}
					else if (item.function.type == eNvFlowPreprocessorType_function)
					{
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_leftParenthesis);
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_rightParenthesis);
					}
					else if (item.function.type == eNvFlowPreprocessorType_attribute)
					{
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_leftBracket);
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_rightBracket);
					}
					else if (item.function.type == eNvFlowPreprocessorType_body)
					{
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_leftCurlyBrace);
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_rightCurlyBrace);
					}
					else if (item.function.type == eNvFlowPreprocessorType_templateInstance)
					{
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_lessThan);
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_greaterThan);
					}
					else if (item.function.type == eNvFlowPreprocessorType_index)
					{
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_leftBracket);
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_rightBracket);

						NvFlowUint64 childTokenEndWithEquals = childTokenEnd;

						NvFlowPreprocessorSkipWhitespaceTokens(&childTokenEndWithEquals, tokenStack_size, tokenStack_data);

						// check for =
						if (childTokenEndWithEquals < tokenStack_size)
						{
							const NvFlowPreprocessorToken token = tokenStack_data[childTokenEndWithEquals];
							if (token.type == eNvFlowPreprocessorTokenType_equals)
							{
								childTokenEndWithEquals++;

								NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEndWithEquals, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_semicolon);

								// commit
								childTokenEnd = childTokenEndWithEquals;
							}
						}
					}
					else if (item.function.type == eNvFlowPreprocessorType_line)
					{
						NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_newline);
					}

					if (!ptr->items[itemIdx].function.allowRecursion)
					{
						ptr->items[itemIdx].isEnabled = NV_FLOW_FALSE;		// disable recursion
					}

					if (item.function.substitute)
					{
						char* substituteStr = item.function.substitute(
							ptr, 
							item.function.userdata, 
							childTokenEnd - childTokenBegin, 
							tokenStack_data + childTokenBegin
						);

						char* substituteOutput = nullptr;
						if (ptr->mode == eNvFlowPreprocessorMode_singlePass)
						{
							substituteOutput = substituteStr;
						}
						else // eNvFlowPreprocessorModeDefault or eNvFlowPreprocessorMode_disable_passthrough
						{
							substituteOutput = NvFlowPreprocessorExecute(ptr, substituteStr);
						}

						ptr->stringStack.pushBack(substituteOutput);
					}

					if (!ptr->items[itemIdx].function.allowRecursion)
					{
						ptr->items[itemIdx].isEnabled = NV_FLOW_TRUE;
					}

					// advance tokenIdx
					if (childTokenEnd > childTokenBegin)
					{
						tokenIdx += childTokenEnd - childTokenBegin - 1u;
					}

					break;
				}
			}
			// If no match found, pass through token
			if (itemIdx == ptr->items.size)
			{
				if (ptr->mode == eNvFlowPreprocessorMode_disable_passthrough)
				{
					// NOP
				}
				else
				{
					ptr->stringStack.pushBack(firstToken.str);
				}
			}
		}
	}

	// pop string stack
	NvFlowUint64 stringStackEnd = ptr->stringStack.size;

	char* ret = NvFlowStringConcatN(ptr->stringPool, ptr->stringStack.data + stringStackBegin, stringStackEnd - stringStackBegin);
	
	ptr->stringStack.size = stringStackBegin;

	// decrement level
	ptr->currentLevel--;

	return ret;
}

char* NvFlowPreprocessorExecuteGlobal(NvFlowPreprocessor* ptr, const char* input, void* userdata, char*(*substitute)(NvFlowPreprocessor* ptr, void* userdata, NvFlowPreprocessorGlobalType globalType, NvFlowUint64 numTokens, const NvFlowPreprocessorToken* tokens))
{
	// increment level
	ptr->currentLevel++;

	NvFlowUint64 stringStackBegin = ptr->stringStack.size;

	// tokenize
	NvFlowPreprocessorToken* tokenStack_data = nullptr;
	NvFlowUint64 tokenStack_size = 0u;
	NvFlowPreprocessorTokenize(ptr, input, &tokenStack_size, &tokenStack_data);

	// process tokens
	NvFlowUint64 tokenIdx = 0u;
	while (tokenIdx < tokenStack_size)
	{
		NvFlowPreprocessorToken firstToken = tokenStack_data[tokenIdx];

		// skip whitespace, but include in output stream
		if (NvFlowPreprocessorTokenIsWhitespace(firstToken))
		{
			ptr->stringStack.pushBack(firstToken.str);
			tokenIdx++;
			continue;
		}

		NvFlowUint64 childTokenBegin = tokenIdx;
		NvFlowUint64 childTokenEnd = tokenIdx;
		
		NvFlowPreprocessorGlobalType globalType = eNvFlowPreprocessorGlobalType_unknown;

		// check for # condition
		if (firstToken.type == eNvFlowPreprocessorTokenType_pound)
		{
			NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_newline);

			globalType = eNvFlowPreprocessorGlobalType_line;
		}
		// check for [ condition
		if (firstToken.type == eNvFlowPreprocessorTokenType_leftBracket)
		{
			NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_leftBracket);
			NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_rightBracket);

			globalType = eNvFlowPreprocessorGlobalType_attribute;
		}
		
		// attempt to detect a function declaration, unless line was detected
		if (childTokenBegin == childTokenEnd)
		{
			// names and whitespace are acceptable up to initial (
			while (childTokenEnd < tokenStack_size)
			{
				const NvFlowPreprocessorToken token = tokenStack_data[childTokenEnd];
				if (!(token.type == eNvFlowPreprocessorTokenType_name || NvFlowPreprocessorTokenIsWhitespace(token)))
				{
					break;
				}
				childTokenEnd++;
			}

			if (childTokenBegin != childTokenEnd && tokenStack_data[childTokenEnd].type == eNvFlowPreprocessorTokenType_leftParenthesis)
			{
				NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_leftParenthesis);
				NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_rightParenthesis);
				NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_leftCurlyBrace);
				NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_rightCurlyBrace);

				globalType = eNvFlowPreprocessorGlobalType_function;
			}
			else
			{
				// invalidate
				childTokenEnd = childTokenBegin;
			}
		}

		// attempt to extract a simple statement
		if (childTokenBegin == childTokenEnd)
		{
			NvFlowPreprocessorExtractTokensDelimited(ptr, &childTokenEnd, tokenStack_size, tokenStack_data, eNvFlowPreprocessorTokenType_semicolon);

			globalType = eNvFlowPreprocessorGlobalType_statement;
		}

		if (childTokenBegin == childTokenEnd)
		{
			// not indentified, force advance
			childTokenEnd++;
		}

		if (globalType != eNvFlowPreprocessorGlobalType_unknown)
		{
			char* substituteOutput = nullptr;
			if (substitute)
			{
				substituteOutput = substitute(ptr, userdata, globalType, childTokenEnd - childTokenBegin, tokenStack_data + childTokenBegin);
			}
			if (substituteOutput)
			{
				ptr->stringStack.pushBack(substituteOutput);
			}
		}
		else
		{
			for (NvFlowUint64 localTokenIdx = childTokenBegin; localTokenIdx < childTokenEnd; localTokenIdx++)
			{
				ptr->stringStack.pushBack(tokenStack_data[localTokenIdx].str);
			}
		}

		// advance tokenIdx
		tokenIdx = childTokenEnd;
	}

	// pop string stack
	NvFlowUint64 stringStackEnd = ptr->stringStack.size;

	char* ret = NvFlowStringConcatN(ptr->stringPool, ptr->stringStack.data + stringStackBegin, stringStackEnd - stringStackBegin);

	ptr->stringStack.size = stringStackBegin;

	// decrement level
	ptr->currentLevel--;

	return ret;
}
