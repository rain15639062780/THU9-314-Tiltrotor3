#!/bin/bash
a
#echo "argc = $#"
echo "argv[1] = $1"

#提取日志编号，创建文件夹
Var=$1

#根据不同日志文件名格式修改1
#Dir=${Var%_*}
#Dir=${Var%*_*}
Dir=$2
echo "dir=$Dir"

mkdir $Dir

#拷贝日志文件，更改文件名（去除具体时间）
cp $1 ./${Dir}/
cd ./${Dir}

#根据不同日志文件名格式修改1
#Fname1=${Var%.*}
#Fname1=${Var%*-*-*-*}
Fname1=$2
echo "Fname1=$Fname1"

Fname2=${Var:0-4:4}
echo "Fname2=$Fname2"

Fname=$Fname1$Fname2
echo "Fname=$Fname"

mv ${Var} ${Fname}

#日志文件格式转换
echo "ulog2csv ${Fname}..."
ulog2csv ${Fname}

ls 

cd ../

echo "creat ${Dir}...Ok"
echo "modify flie name...Ok"
echo "ulog2csv ${Fname}...Ok"

if [ $3 = "d" -o $3 = "D" ]
	then	
	rm $1
	echo "$1 already delete"
	else
	echo "if want delete $1,please input -d or -D"
	fi



#rm ${Dir} -R

