
void WriteLineData()
	{
		//int lineNum = 2;
		ifstream infile;
		//string file;
		char file[] =  "/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/src/mission.txt";
		char file1[] =  "/home/youibot/mrobot/src/mmrobot/mm_control/mm_robot_decision/src/mission1.txt";
		infile.open(file);

		char linetxt[1024];
		int numOfLine = 1;
		while (infile.getline(linetxt, 1024))
		{
			if (lineNum == numOfLine)
			{
				break;
			}
			numOfLine++;
		}

		strcat(linetxt, "success");


		string strFileData = "";
		int line = 1;
		char tmpLineData[1024] = { 0 };
		while (infile.getline(tmpLineData, sizeof(tmpLineData)))
		{
			if (line == lineNum)
			{
				strFileData += CharToStr(linetxt);
				strFileData += "\n";
			}
			else
			{
				strFileData += CharToStr(tmpLineData);
				strFileData += "\n";
			}
			line++;
		}
		infile.close();

		//写入文件
		ofstream out;
		out.open(file1);
		out.flush();
		out << strFileData;
		out.close();
	}
