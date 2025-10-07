#include <File.h>

bool File::Exists(const std::string& filename)
{
	return std::filesystem::exists(filename);
}

File::File()
	: m_pFileStream(nullptr)
{
	m_pFileStream = new std::fstream();
}

File::File(const std::string& fileName, bool isBinary)
	: m_fileName(fileName)
{
	if (Exists(fileName))
	{
		m_pFileStream = new std::fstream();

		Open(fileName, isBinary);
	}
}

File::~File()
{
	if (m_pFileStream != nullptr) {
		delete m_pFileStream;
	}
}

void File::Create(const std::string& fileName, bool isBinary)
{
	if (isBinary) {
		(*m_pFileStream).open(fileName, std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);
	}
	else {
		(*m_pFileStream).open(fileName, std::ios::in | std::ios::out | std::ios::trunc);
	}
}

bool File::Open(const std::string& fileName, bool isBinary)
{
	m_fileName = fileName;
	if (false == Exists(fileName)) return false;

	if (isBinary) {
		(*m_pFileStream).open(fileName, std::ios::binary | std::ios::in);
	}
	else {
		(*m_pFileStream).open(fileName, std::ios::in);
	}

	if ((*m_pFileStream).is_open()) {
		(*m_pFileStream).seekg(0, std::ios::end);
		m_fileLength = int((*m_pFileStream).tellg());
		(*m_pFileStream).seekg(0, std::ios::beg);

		return true;
	}

	return false;
}

void File::Close()
{
	(*m_pFileStream).close();
}

bool File::isOpen()
{
	if (nullptr == m_pFileStream) return false;

	return (*m_pFileStream).is_open();
}

bool File::GetWord(std::string& word)
{
	if (nullptr == m_pFileStream) return false;

	return !((*m_pFileStream) >> word).eof();
}

bool File::GetLine(std::string& line)
{
	if (nullptr == m_pFileStream) return false;

	return !(getline((*m_pFileStream), line).eof());
}

void File::Read(char* buffer, int length) const
{
	if (nullptr == m_pFileStream) return;

	(*m_pFileStream).read(buffer, length);
}

void File::Write(char* buffer, int length)
{
	if (nullptr == m_pFileStream) return;

	(*m_pFileStream).write(buffer, length);
}

std::fstream& File::operator << (bool data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (short data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (unsigned short data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (int data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (unsigned int data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (long data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (unsigned long data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (float data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (double data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (std::string& data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (const std::string& data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (char* data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

std::fstream& File::operator << (const char* data)
{
	return (std::fstream&)((*m_pFileStream) << data);
}

//

std::fstream& File::operator >> (bool& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (short& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (unsigned short& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (int& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (unsigned int& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (long& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (unsigned long& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (float& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (double& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

std::fstream& File::operator >> (std::string& data)
{
	return (std::fstream&)((*m_pFileStream) >> data);
}

//std::fstream& File::operator >> (char* data)
//{
//	return (std::fstream&)((*m_pFileStream) >> data);
//}
