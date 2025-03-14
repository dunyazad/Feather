#include <File.h>

bool File::Exists(const string& filename)
{
	return filesystem::exists(filename);
}

File::File()
	: m_pFileStream(nullptr)
{
	m_pFileStream = new fstream();
}

File::File(const string& fileName, bool isBinary)
	: m_fileName(fileName)
{
	if (Exists(fileName))
	{
		m_pFileStream = new fstream();

		Open(fileName, isBinary);
	}
}

File::~File()
{
	if (m_pFileStream != nullptr) {
		delete m_pFileStream;
	}
}

void File::Create(const string& fileName, bool isBinary)
{
	if (isBinary) {
		(*m_pFileStream).open(fileName, ios::binary | ios::in | ios::out | ios::trunc);
	}
	else {
		(*m_pFileStream).open(fileName, ios::in | ios::out | ios::trunc);
	}
}

bool File::Open(const string& fileName, bool isBinary)
{
	m_fileName = fileName;
	if (false == Exists(fileName)) return false;

	if (isBinary) {
		(*m_pFileStream).open(fileName, ios::binary | ios::in);
	}
	else {
		(*m_pFileStream).open(fileName, ios::in);
	}

	if ((*m_pFileStream).is_open()) {
		(*m_pFileStream).seekg(0, ios::end);
		m_fileLength = int((*m_pFileStream).tellg());
		(*m_pFileStream).seekg(0, ios::beg);

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

bool File::GetWord(string& word)
{
	if (nullptr == m_pFileStream) return false;

	return !((*m_pFileStream) >> word).eof();
}

bool File::GetLine(string& line)
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

fstream& File::operator << (bool data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (short data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (unsigned short data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (int data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (unsigned int data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (long data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (unsigned long data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (float data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (double data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (string& data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (const string& data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (char* data)
{
	return (fstream&)((*m_pFileStream) << data);
}

fstream& File::operator << (const char* data)
{
	return (fstream&)((*m_pFileStream) << data);
}

//

fstream& File::operator >> (bool& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (short& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (unsigned short& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (int& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (unsigned int& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (long& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (unsigned long& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (float& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (double& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

fstream& File::operator >> (string& data)
{
	return (fstream&)((*m_pFileStream) >> data);
}

//fstream& File::operator >> (char* data)
//{
//	return (fstream&)((*m_pFileStream) >> data);
//}
