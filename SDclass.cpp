#include "SDclass.h"

char REDpath[13] = "PATH_RED.txt";
char REDvel[13] = "VEL_RED.txt";
char BLUpath[13] = "PATH_BLU.txt";
char BLUvel[13] = "VEL_BLU.txt";

mySDclass::mySDclass(){
}

/* SDカードの初期化 */
int mySDclass::init(){
  if (SD.begin()) {
    SD_enable = true;
    return 0;
  }
  return -1;
}

/* ログデータ書き出し用ファイル名の設定 */
int mySDclass::make_logfile(){
  bool nameOK = false;
  int file_num = 0;

  while(!nameOK && file_num < 10000){
    logFileName = "MR1_";
    if(file_num < 10){
      logFileName += "000";
      logFileName += String(file_num);
    }else if(file_num < 100){
      logFileName += "00";
      logFileName += String(file_num);
    }else if(file_num < 1000){
      logFileName += "0";
      logFileName += String(file_num);
    }else{
      logFileName += String(file_num);
    }
    
    logFileName += ".txt";
    logFileName.toCharArray(c_logFileName, 13 );
    if(SD.exists(c_logFileName)){
      file_num++;
    }
    else{
      nameOK = true;
    }
  }
  return 0;
}

/* ログデータ書き出し用の関数 */
int mySDclass::write_logdata(String dataString){
  File dataFile = SD.open(c_logFileName, FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    //Serial.println(dataString);
    return 0;
  }  
  else {
    //Serial.println("error opening ");
    return -1;
  }
}

int mySDclass::path_read(int field, double Px[], double Py[], double vel[], double angle[]){
  File myFile;
  char *pathFile;
  char *velFile;
  char tmpchar;
  String tmpA = "", tmpB = "";
  bool file_end  = false;

  // 赤か青かで読み込むファイルを変更する
  if(field == RED){
    pathFile = REDpath; // 赤ゾーンのパス設定ファイル
    velFile = REDvel; // 赤ゾーンの速度と角度の設定ファイル
  }else if(field == BLUE){
    pathFile = BLUpath; // 青ゾーンのパス設定ファイル
    velFile = BLUvel; // 青ゾーンの速度と角度の設定ファイル
  }else{
    return -1;
  }

  // パス設定用ファイルからデータを読み込む
  myFile = SD.open(pathFile, FILE_READ);
  if (myFile) {
    // read from the file until there's nothing else in it:
    while (!file_end && myFile.available()) {
      while((tmpchar = myFile.read()) != ','){
        tmpA += tmpchar;
      }
      *Px = (double)tmpA.toFloat();
      Serial.print(tmpA);
      tmpA = "";
      Px++;
      while((tmpchar = myFile.read()) != '\r' && tmpchar != ';'){
        tmpB += tmpchar;
      }
      if(tmpchar == ';'){
        file_end = true;
      }else{
        myFile.read(); // "\n"を捨てるため
      }
      *Py = (double)tmpB.toFloat();
      Serial.print(tmpB);
      tmpB = "";
      Py++;
    }
    //Serial.print("path done! ");
    file_end = false;
    // close the file:
     myFile.close();
  } else {
  	// if the file didn't open, print an error:
    //Serial.println("error opening test.txt");
    return -2;
  }

  // 速度設定用ファイルからデータを読み込む
  myFile = SD.open(velFile, FILE_READ);
  if (myFile) {
    while (!file_end && myFile.available()) {
      while((tmpchar = myFile.read()) != ','){
        tmpA += tmpchar;
      }
      *vel = (double)tmpA.toFloat();
      Serial.print(tmpA);
      tmpA = "";
      vel++;
      while((tmpchar = myFile.read()) != '\r' && tmpchar != ';'){
        tmpB += tmpchar;
      }
      if(tmpchar == ';'){
        file_end = true;
      }else{
        myFile.read(); // "\n"を捨てるため
      }
      *angle = (double)tmpB.toFloat();
      Serial.print(tmpB);
      tmpB = "";
      angle++;
    }
    //Serial.println("vel/angle done!");
    // close the file:
     myFile.close();
  } else {
  	// if the file didn't open, print an error:
    //Serial.println("error opening test.txt");
    return -3;
  }

  return 0;
}
