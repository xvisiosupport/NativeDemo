//=============================================================================
// FILE: svrApiConfig.cpp
//                  Copyright (c) 2016 QUALCOMM Technologies Inc.
//                              All Rights Reserved.
//
//==============================================================================
#include "glm/glm.hpp"

#include "svrUtil.h"
#include "svrConfig.h"

static Map<VariableBase>* sVariableMap;

static Map<VariableBase>* VariableMap()
{
	if (!sVariableMap) sVariableMap = new Map<VariableBase>();
	return sVariableMap;
}

VariableBase* GetVariable(const char* name)
{
	return VariableMap()->Find(name);
}

bool AddVariable(VariableBase* variable)
{
	return VariableMap()->Insert(variable);
}

static int GetWhitespaceLength(const char* text)
{
	const char* start = text;
	for (;;) {
		for (;;) {
			unsigned long c = *reinterpret_cast<const unsigned char*>(text);
			if (c == 0) goto end;

			if (c > 32) {
				if ((c != '/') || (text[1] != '/')) goto end;
				break;
			}

			text++;
		}

		text += 2;
		for (;;) {
			unsigned long c = *reinterpret_cast<const unsigned char*>(text);
			if (c == 0) goto end;

			text++;
			if ((c == 10) || (c == 13)) break;
		}
	}

end:
	return (int)(text - start);
}

static int ReadIdentifier(const char* text, char* identifier, long max)
{
	const char* start = text;
	while (--max >= 0) {
		unsigned long c = *reinterpret_cast<const unsigned char*>(text);
		if ((c - '0' >= 10U) && (c - 'A' >= 26U) && (c - 'a' >= 26U) && (c != '_') && (c != '$')) break;

		*identifier++ = (char)c;
		text++;
	}

	*identifier = 0;
	return (int)(text - start);
}

static int FindUnquotedChar(const char* text, unsigned long k)
{
	bool quote = false;
	bool backslash = false;

	const char* start = text;
	for (;;) {
		unsigned long c = *reinterpret_cast<const unsigned char*>(text);
		if (c == 0) break;

		if (c == 34) {
			if (!quote) quote = true;
			else if (!backslash) quote = false;
		}

		if ((c == k) && (!quote)) return (int)(text - start);

		backslash = ((c == 92) && (!backslash));
		text++;
	}

	return (-1);
}

static int ReadString(const char* text, char* string, long max)
{
	const char* start = text;
	--max;

	if (*text == 34) {
		text++;
		bool backslash = false;

		while (--max >= 0) {
			unsigned long c = *reinterpret_cast<const unsigned char*>(text);
			if (c == 0) break;

			text++;

			if ((c != 92) || (backslash)) {
				if ((c == 34) && (!backslash)) break;
				*string++ = (char)c;
				backslash = false;
			}
			else {
				backslash = true;
			}
		}
	}
	else {
		while (--max >= 0) {
			unsigned long c = *reinterpret_cast<const unsigned char*>(text);
			if ((c == 0) || (c < 33) || ((c == '/') && (text[1] == '/'))) break;

			*string++ = (char)c;
			text++;
		}
	}

	*string = 0;
	return (int)(text - start);
}

void LoadVariable(const char* text)
{
	char name[kMaxVariableValueLength];

	text += ReadIdentifier(text, name, kMaxVariableValueLength);
	text += GetWhitespaceLength(text);

	VariableBase* variable = GetVariable(name);
//	LOGI("eddy LoadVariable %s = %s", name);
	if (*text == '=' && variable) {
		char value[kMaxVariableValueLength];

		text++;
		text += GetWhitespaceLength(text);
		ReadString(text, value, kMaxVariableValueLength);
		variable->SetValue(value);
		LOGI("%s = %s", name, value);
	}
	else {
		if (variable) {
			char value[kMaxVariableValueLength];
			variable->GetValue(value, kMaxVariableValueLength);
			LOGI("%s = %s", name, value);
		}
		else {
			LOGE("Undefined variable -- %s", name);
		}
	}
}

void LoadCommandLineVariables(char* argv[], int argc)
{
	for (int i = 1; i<argc;) {
		VariableBase* variable = GetVariable(argv[i]);
		if (i + 1 < argc && variable) {
			char value[kMaxVariableValueLength];
			ReadString(argv[i + 1], value, kMaxVariableValueLength);
			variable->SetValue(value);
			i += 2;
			continue;
		}
		++i;
	}
}

void LoadVariableBuffer(const char* buffer)
{
    char line[kMaxVariableValueLength * 2];
	LOGE("LoadVariableBuffer entry");
    const char* text = buffer;
    for (;;) 
    {
		LOGE("LoadVariableBuffer entry 1");
        memset(line, 0, kMaxVariableValueLength * 2);
        text += GetWhitespaceLength(text);
        if (*text == 0) return;

        long length = FindUnquotedChar(text, '\n');
        if (length < 0) 
        {
            strncpy(line, text, kMaxVariableValueLength * 2);
            LoadVariable(line);
            break;
        }

        if (length > 0) 
        {
            strncpy(line, text, glm::min(length, long(kMaxVariableValueLength * 2)));
            LoadVariable(line);
        }

        text += length + 1;
    }
}

void LoadVariableFile(const char* filename)
{
	LOGI("Opening variables file: %s", filename);
	FILE* fp = fopen(filename, "r");
	if (!fp) {
		LOGE("Unable to open variables file: %s", filename);
		return;
	}

	fseek(fp, 0, SEEK_END);
	int size = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	char* buf = new char[size + 1];
	fread(buf, 1, size, fp);
	buf[size] = 0;

	fclose(fp);

    LoadVariableBuffer(buf);

	delete[] buf;
}

void WriteVariableFile(const char* filename)
{
//	LOGI("WriteVariableFile variables file: %s", filename);
	FILE* fp = fopen(filename, "w");
	if (!fp) {
		LOGE("Unable to open variables file: %s", filename);
		return;
	}

	VariableBase* variable = VariableMap()->First();
	while (variable) {
		if (!(variable->GetFlags() & kVariableNonpersistent)) {
			char value[kMaxVariableValueLength];
			variable->GetValue(value, kMaxVariableValueLength);
			char buffer[kMaxVariableValueLength * 2];
			sprintf(buffer, "%s = %s\n", variable->GetName(), value);
			fwrite(buffer, 1, strlen(buffer), fp);
       //     LOGI("WriteVariableFile variables  value %s",value);
		}
    //    LOGI("WriteVariableFile variables %s",  variable->GetName());
		variable = variable->Next();
	}
    fclose(fp);
}