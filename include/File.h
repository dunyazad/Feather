#pragma once

#include <FeatherCommon.h>

class File
{
public:
	File();
	File(const std::string& fileName, bool isBinary = false);
	~File();

	static bool Exists(const std::string& filename);

	void Create(const std::string& fileName, bool isBinary);
	bool Open(const std::string& fileName, bool isBinary);
	void Close();
	bool isOpen();

	bool GetWord(std::string& word);
	bool GetLine(std::string& line);

	void Read(char* buffer, int length) const;
	void Write(char* buffer, int length);

	std::fstream& operator << (bool data);
	std::fstream& operator << (short data);
	std::fstream& operator << (unsigned short data);
	std::fstream& operator << (int data);
	std::fstream& operator << (unsigned int data);
	std::fstream& operator << (long data);
	std::fstream& operator << (unsigned long data);
	std::fstream& operator << (float data);
	std::fstream& operator << (double data);
	std::fstream& operator << (std::string& data);
	std::fstream& operator << (const std::string& data);
	std::fstream& operator << (char* data);
	std::fstream& operator << (const char* data);

	std::fstream& operator >> (bool& data);
	std::fstream& operator >> (short& data);
	std::fstream& operator >> (unsigned short& data);
	std::fstream& operator >> (int& data);
	std::fstream& operator >> (unsigned int& data);
	std::fstream& operator >> (long& data);
	std::fstream& operator >> (unsigned long& data);
	std::fstream& operator >> (float& data);
	std::fstream& operator >> (double& data);
	std::fstream& operator >> (std::string& data);
	//std::fstream& operator >> (char* data);

	inline const std::string& GetFileName() const { return m_fileName; }
	inline int GetFileLength() const { return m_fileLength; }

private:
	std::fstream* m_pFileStream = nullptr;
	std::string m_fileName = "";
	int m_fileLength = 0;
};
