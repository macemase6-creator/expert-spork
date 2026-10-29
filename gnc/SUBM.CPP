#include"stdafx.h"
#include"M273N.H"
int popup( char *matn[],char *keys,int count,int x,int y)// ,int border)
{

	return nchoice;

}
//#include "M273N.H"
//#include "alloc.h"
//#include "dir.h"
//#include<string.h>
//#include <stdio.h>
//#include <dos.h>
//#include <stdlib.h>
//#include <conio.h>
//#include <ctype.h>

//#include "alloc.h"
//#include "dir.h"

/*#define  border           1
#define  ESC             27
#define  REV_VID       0x70
#define  NORM_VID         7*/

/*typedef unsigned int uint;
typedef unsigned char uch;
int get_key();
int bioskey(int c);
int is_in(char *s,char c);
void write_string(int x,int y,char *p,int attrib);
void drow_border(int startx,int starty, int endx,int endy);
int get_choice(int x,int y,int count,char *matn[],char *keys);
void get_data(int ndim,char *filename,double data[]);
void display_matn(char *matn[],int x,int y,int count)  ;
void write_video(int x,int y,char *p,int attrib);*/


/*int popup( char *matn[],char *keys,int count,int x,int y)// ,int border)
	    {
	     int i,len;
	     int endx,endy,choice;
	     if((x>24) || (x<0) || (y>79) || (y<0) )
	     {
	      printf("range");
	      return -2 ;
	     }

	    /* compute the dimesions . 
	     len=0;
	     for(i=0;i<count ;i++)
		if(strlen(matn[i])>len)
		len=strlen(matn[i]);

	     endy=len+2+y;
	     endx=count+1+x;
	     if((endx+1 >24) || (endy+1 >79))
	      {
	       printf("matn won't fit ");
	       return -2;
	      }
	    drow_border(x,y,endx,endy) ;
	    display_matn(matn,x+1,y+1,count);
	    choice=get_choice(x+1,y,count,matn,keys) ;
	    return choice;
	 }

void display_matn(char *matn[],int x,int y,int count)
	{
	 register int i ;
         for(i=0;i<count ;i++,x++)
          {
	  gotoxy(y,x);
	  printf(matn[i]);
          }
        }

void drow_border(int startx,int starty,int endx,int endy){

                   register int i ;
             for (i=startx+1 ; i<endx ;i++)
              {
	      gotoxy(starty,i) ;
	      putchar(179);
	      gotoxy(endy,i);
	      putchar(179);
	      }

	     for (i=starty+1 ; i<endy ;i++)
	      {
	      gotoxy(i,startx) ;
	      putchar(196);
	      gotoxy(i,endx);
              putchar(196);
              }

	      gotoxy(starty,startx) ;
	      putchar(218);
	      gotoxy(endy,startx) ;
	      putchar(191);
	      gotoxy(starty,endx) ;
	      putchar(192);
	      gotoxy(endy,endx) ;
              putchar(217);
          }



int get_choice(int x,int y,int count,char *matn[],char *keys)


              {
               union inkey {
                            char ch[2];
                            int i;
                            } c ;
              int arrow_choice = 0 ,key_choice ;
              y++;
	      gotoxy(y,x);
              write_video(x,y,matn[0],REV_VID) ;

              for(;;)
                 {


                 while(!bioskey(1));
                 c.i=bioskey(0);
		 gotoxy(y,x+arrow_choice);
                 write_video(x+arrow_choice,y,
                 matn[arrow_choice],NORM_VID) ;
                 if(c.ch[0])
                  {
                   key_choice=is_in(keys,tolower(c.ch[0]));
                   if(key_choice)
		     return key_choice-1;
		     if(c.ch[0]>'0'&& c.ch[0]<='9') arrow_choice=c.ch[0]-'1';
		     else
		     switch(c.ch[0])
		      {
		      case '\r' : return arrow_choice ;
		      case ' '  : arrow_choice++ ;break;
		      case ESC  : return -1;
		      }
		  }
		 else
		  {
		   switch(c.ch[1])
		     {
		      case 72 : arrow_choice -- ;
			break;
		      case 80 : arrow_choice ++;
			break;
		     }
		  }
		  if(arrow_choice==count)
		    arrow_choice =0;
		  if(arrow_choice<0)
		   arrow_choice=count-1;
		  gotoxy(y,x+arrow_choice);
		  write_video(x+arrow_choice,y,matn[arrow_choice],REV_VID);
		 }

	      }


int bioskey(int c)
     {
		 switch(c){
		  case 0 : return get_key();
		  case 1 : return kbhit();
		 }
	  }

 /* read 16 bit scan code . */

/*int get_key()
	{
	 union REGS r;
	 r.h.ah=0;
	 return int86(0x16,&r,&r);
	}



void write_string(int x,int y,char *p,int attrib)
		  {
			register int i;
	 union REGS r;
         for(i=y; *p;i++)
           {
	    gotoxy(i,x);
	    r.h.ah=9;
	    r.x.cx=1;
            r.h.al=*p++;
            r.h.bl=attrib;
            int86(0x10,&r,&r);
           }
	}

int is_in(char *s,char c)
       {
	register int i;
	for(i=0; *s ;i++)
	    if(*s++==c)return i+1;
	    return 0;
       }


void write_video(int x,int y,char *p,int attrib)
{
 register int i;
 union REGS r;
 for(i=y; *p;i++)
  {
  gotoxy(i,x);
  r.h.ah=9;
  r.h.bh=0;
  r.x.cx=1;
  r.h.al=*p++;
  r.h.bl=attrib;
  int86(0x10,&r,&r);
  }
}

void get_data(int dim,char *filename,double data[])
   {
   int i;
   char c[40][60],ans1[25];
   FILE *fdat;
   clrscr();
   printf("\x1B[2;30f Enter the new data.  ");
   printf("\x1B[3;14f If you want save some default data press Enter key.\n");

   fdat=sim_fopen(filename,"r");
    fgets(c[1],256,fdat);
    printf("               %s",c[1]);
    for(i=0;i<dim;i++)
	{

      fgets(c[i],256,fdat);
      c[i][strlen(c[i])-1]=0;

      printf("\x1B[44;32;1m%s",c[i]);

	 printf("\x1B[0m(%14.8e)",data[i]);

      gets(ans1);

     if(ans1[0]!=0)data[i]=atof(ans1);
    }
sim_fclose(fdat);
}

void err(int ierr)
{
switch(ierr)
  {
  case 0:
  return;
  case 1:
  clrscr();
  gotoxy(20,20);
  printf(" ++++++Cannot open file .  ++++++ ");
  exit(1);
  case 2:
  clrscr();
  gotoxy(20,20);
  printf("+++++++Cannot read file .  ++++++  ");
  exit(1);
 }
}*/

void get_data(int ndim,char *filename,double data[])
{


}

void readfile(char piam[],char def[],int dim,char ans[],double data[],int mod)
{
	FILE *f1;
	int i,k;
//	char ans[80]="\0";
	//clrscr();
	/*if(!mod)
	 {
	 // gotoxy(10,6);
	   printf(" %s  [ %s ] :",piam,def);
	  gets(ans);
	  }*/
	strcpy(ans,def);
	f1=sim_fopen(def,"r");
	//if(ans[0]==0)strcpy(ans,def);
	/*if((f1=sim_fopen(def,"r"))==NULL)
	   {
	   // gotoxy(20,20);
	    printf("\  ++++++   Cannot  Open  File . ++++++");
	    exit(0);
	   }*/

	for(i=0;i<dim;i++)
	  fscanf(f1,"%d%lf",&k,&data[i]);
	sim_fclose(f1);
}

void r2dms(double acc, double R, int* pd, int* pm, double* ps)
/*  ---------------------------------------------------------------------
DESCRIPTION : THIS SUB. CONVERT RADIAN VALUE TO DEGREE VALUE
---------------------------------------------------------------------
INPUTS (acc)   : ACCURACY IN PRINTING RADIAN TO DEGREE CONVERSION
(R)     : RADIAN VALUE
OUTPUTS  *pd (ID)  : DEGREES OF ARC
*pm (M)   : MINUTES
*ps (S)   : SECONDS
--------------------------------------------------------------------- */
{
	double  S, RD, RM;
	int ID, M;

	RD = R / d2r;
	ID = RD;
	RM = (RD - ID) * 60.;
	M = RM;
	S = (RM - M) * 60.;
	if (acc > 0. && acc < 0.1)
	{
		if (S + acc - 60. > 0)
		{
			S = 0.;
			++M;
		}
		if (M == 60)
		{
			M = 0;
			++ID;
		}
	}
	*pd = ID;
	*pm = M;
	*ps = S;
	return;
}
//===========================================================================


double dms2r(int D, int M, double S)
/* This function converts the degree,minutes, and seconds value to radian. */
{
	double R;
	R = ((D * 60. + M) * 60. + S) / d2r;
	return(R);
}

//============================================================================

//=============================================================================

int time2show(double t, double incr)
{
	double a = floor(t / incr + 1.e-5);
	if (fabs(t - a * incr) < delt - 1.e-5)
		return 1;

	return 0;
}


//===========================================================================
//=============================================================================
void saveit(double* O_vec)
{
	/*
	PURPOSE   To save the simulation data. 
	The data are written to a file as the simulation progress.

	By Mohammad-Ali Massoumnia
	Farvardin 1376 */


	fwrite(O_vec, sizeof(double), N_to_save, Outfile);

	++Place;

}
//=============================================================================
void saveit2(double* O_vec,FILE *Outfile )
{

	fwrite(O_vec, sizeof(double), N_to_save, Outfile);

	++Place2;

}

int ReadWayPoints(const char* filename)
{
	FILE*  f ;
	int tempi,i;
	float lat1,lat2,lat3,long1,long2,long3,alt;
	char en1[1];
	char en2[1];
	char s[200];

	int index[wp_max];
	int counter = 0,cnt=0;
	for(i=0;i<wp_max;i++) index[i]=-1;

	if ((f = fopen (filename, "r")) == NULL)
	{
		cout << filename<< " not find!"<<endl;
		return -1;
	}
	do
	{
		fgets(s,200,f);
		if(strncmp(s,"DESCRIPTION=Waypoint",20)==0)
		{
			break;
		}
	} while(!feof(f));

	if(feof(f))
		{
			cout << "Incorrect file!"<<endl;
			return -2 ;
		}

	while(!feof(f) && strncmp(s,"DESCRIPTION=Waypoint",20)==0)
	{
		fscanf(f, "NAME=%d\n", &tempi);
		fgets(s,200,f);
		//cnt++;
		//tempi=cnt;
		if(tempi<=wp_max && tempi>0)
		{
			counter++;
			index[tempi-1]=tempi;

			sscanf(s,"%f%*s %f%*s %f%*s %1s,%f%*s %f%*s %f%*s %1s,%f", &lat1, &lat2, &lat3, en1, &long1, &long2, &long3, en2, &alt);
			if(en1[0] == 'N')
			{
				//cout<<tempi<<": "<<lat1<<" "<<lat2<<" "<<lat3<<"  "<<long1<<" "<<long2<<" "<<long3<<"  "<<alt<<endl;
				Latt0[tempi-1]=lat1+(lat2/60)+(lat3/3600);
				Longt0[tempi-1]=long1+(long2/60)+(long3/3600);

				Ht0[tempi-1]=alt;
				Latt0[tempi-1] *=d2r;
				Longt0[tempi-1]*=d2r;
			}
			else
			{
				//cout<<tempi<<": "<<long1<<" "<<long2<<" "<<long3<<"  "<<lat1<<" "<<lat2<<" "<<lat3<<"  "<<alt<<endl;
				Longt0[tempi-1] =long1+(long2/60)+(long3/3600);;
				Latt0[tempi-1]=lat1+(lat2/60)+(lat3/3600);
				Ht0[tempi-1]  =alt;
				Latt0[tempi-1] *=d2r;
				Longt0[tempi-1]*=d2r;

			}
	  // */
			fgets(s,200,f);
			fgets(s,200,f);
		}
		else
		{
			cout << counter <<" points readed from "<<filename<<endl;
			cout << "Incorrect file!"<<endl;
			return -2 ;
		}
	}
	sim_fclose(f);
//// Elahi Shokert 960729 emad
	for(i=0;i<counter;i++)//counter = number of points 
	{
		if(index[i]!=(i+1))
		{
			cout << (i+1) <<" points readed from "<<filename<<endl;
			cout << "Incorrect file!"<<endl;
			return -2;
		}
	}
	return counter;
}

int ReadWayPoints_2(const char* filename)
{
	FILE*  f ;
	int tempi,i;
	float lat1,lat2,lat3,long1,long2,long3,alt;
	char en1[1];
	char en2[1];
	char s[200];

	int index[wp_max];
	int counter = 0,cnt=0;
	for(i=0;i<wp_max;i++) index[i]=-1;

	if ((f = fopen (filename, "r")) == NULL)
	{
		cout << filename<< " not find!"<<endl;
		return -1;
	}
	do
	{
		fgets(s,200,f);
		if(strncmp(s,"DESCRIPTION=Waypoint",20)==0)
		{
			break;
		}
	} while(!feof(f));

	if(feof(f))
		{
			cout << "Incorrect file!"<<endl;
			return -2 ;
		}

	while(!feof(f) && strncmp(s,"DESCRIPTION=Waypoint",20)==0)
	{
		//fscanf(f, "NAME=%d\n", &tempi);
		fgets(s,200,f);
		cnt++;
		tempi=cnt;
		if(tempi<=wp_max && tempi>0)
		{
			counter++;
			index[tempi-1]=tempi;

			sscanf(s,"%f%*s %f%*s %f%*s %1s,%f%*s %f%*s %f%*s %1s,%f", &lat1, &lat2, &lat3, en1, &long1, &long2, &long3, en2, &alt);
			if(en1[0] == 'N')
			{
				//cout<<tempi<<": "<<lat1<<" "<<lat2<<" "<<lat3<<"  "<<long1<<" "<<long2<<" "<<long3<<"  "<<alt<<endl;
				Latt0[tempi-1]=lat1+(lat2/60)+(lat3/3600);
				Longt0[tempi-1]=long1+(long2/60)+(long3/3600);

				Ht0[tempi-1]=alt;
				Latt0[tempi-1] *=d2r;
				Longt0[tempi-1]*=d2r;
			}
			else
			{
				//cout<<tempi<<": "<<long1<<" "<<long2<<" "<<long3<<"  "<<lat1<<" "<<lat2<<" "<<lat3<<"  "<<alt<<endl;
				Longt0[tempi-1] =long1+(long2/60)+(long3/3600);;
				Latt0[tempi-1]=lat1+(lat2/60)+(lat3/3600);
				Ht0[tempi-1]  =alt;
				Latt0[tempi-1] *=d2r;
				Longt0[tempi-1]*=d2r;

			}
	  // */
			fgets(s,200,f);
			fgets(s,200,f);
		}
		else
		{
			cout << counter <<" points readed from "<<filename<<endl;
			cout << "Incorrect file!"<<endl;
			return -2 ;
		}
	}
	sim_fclose(f);
//// Elahi Shokert 960729 emad
	for(i=0;i<counter;i++)//counter = number of points 
	{
		if(index[i]!=(i+1))
		{
			cout << (i+1) <<" points readed from "<<filename<<endl;
			cout << "Incorrect file!"<<endl;
			return -2;
		}
	}
	return counter;
}




//char * CreateForderPath(int foldername)
char * CreateForderPath(int Lfoldername,int Tfoldername,int hgvflg)
{
	char folderpath0[60];
	static char folderpath[260];
	/*sprintf(folderpath0,"HGV");
	
		if(_mkdir(folderpath0)==-1)
	{
		if(errno !=EEXIST)
		{
		//perror("error creating dir");
			printf("\n\error creating dir: %s\n ",folderpath);
			exit(1);

		//return NULL;
		}
		
	}//*/
	
	//sprintf(folderpath,"%d",foldername);
	/* tal_flag=1 flies a PARALLEL scenario-table set (HL<l>T<t>_TAL): the
	   Edit-4 vehicle is heavier and 24% more energetic, so the generator
	   produces a different boost program. Reusing the stock tables would
	   leave the reference attitude program stale against the flown vehicle
	   -- the documented silent-failure mode. flag=0 yields the identical
	   string, so the stock path is untouched. */
	if(hgvflg)	sprintf(folderpath,"HL%dT%d%s",Lfoldername,Tfoldername,sim_tal_flag?"_TAL":"");
	else 		sprintf(folderpath,"BL%dT%d%s",Lfoldername,Tfoldername,sim_tal_flag?"_TAL":"");

	//sprintf(folderpath,"TrgtCnt_%d",foldername);
	
	if(_mkdir(folderpath)==-1)
	{
		if(errno !=EEXIST)
		{
		//perror("error creating dir");
			printf("\n\error creating dir: %s\n ",folderpath);
			exit(1);

		//return NULL;
		}
		
	}//*/	
	
	return folderpath;

}

char * CearteFoFiPath(int foldername,char *filename)
{
	char folderpath[60];
	static char filepath[100];
	
	sprintf(folderpath,"%d",foldername);
	
	if(_mkdir(folderpath)==-1)
	{
		if(errno !=EEXIST)
		{
		//perror("error creating dir");
			printf("\n\error creating dir\n ");
			exit(1);

		//return NULL;
		}
		
	}
	
	sprintf(filepath,"%s\\%s",folderpath,filename);
	
	
	return filepath;

}

char * CearteFilePath( char *filename, char *folderpath)
{
	//char folderP[60];
	static char filepath[260];
	
	sprintf(filepath,"%s\\%s",folderpath,filename);
	
	return filepath;
	

}
char * getFilePath( char *filename, char *folderpath)
{
	//char folderP[60];
	static char filepath[260];
	
	sprintf(filepath,"%s\\%s",folderpath,filename);
	
	return filepath;
	

}

//int CopyFile(int foldername,char *sourcefilename,int mod)
int CopyFile(char *destfoldername,char *sourcefilename,int mod)
{
	char destFilePath[260];
	
	static char filepath[100];
	char destFolderPath[100];
	
	FILE *sourceFile,*destFile;
	char buffer[1024];
	size_t bytesRead;
	
	/*
	char destfoldername[60];
	sprintf(destfoldername,"%d",foldername);
	
	if(_mkdir(destfoldername)==-1)
	{
		if(errno !=EEXIST)
		{
		//perror("error creating dir");
			printf("\n\error creating dir\n ");
			exit(1);

		//return NULL;
		}
		
	}//*/
	
	const char *filename = strrchr(sourcefilename,'\\');
	
	if(!filename)
	{
		filename = strrchr(sourcefilename,'/');
	}	
	if(filename)
	{
		filename ++;
	}else
	{
		filename = sourcefilename;
	}
	

	//sprintf(destFolderPath,"%s\\%s",destfoldername,filename);//*/

	if(mod==0)
	{
		char clearFileN[100];
		strncpy(clearFileN,filename,sizeof(clearFileN)-1);
		clearFileN[sizeof(clearFileN)-1] ='\0';
		
		sprintf(destFilePath,"%s\\%s",destfoldername,clearFileN);
		//strcpy(destFilePath,destFolderPath);
	}
	//else if(mod==1)
	//{
	//	sprintf(destFolderPath,"%s\\%s",destfoldername,filename);
	//	strcpy(destFilePath,destFolderPath);
	//}
	else
		sprintf(destFilePath,"%s\\%s",destfoldername,filename);

	sourceFile=sim_fopen(sourcefilename,"rb");
	
	if (sourceFile==NULL)
	{
		perror("error creating file");
		printf("\n\error opening source file: %s\n ",sourceFile);
		return 0;
	}
	
	destFile=sim_fopen(destFilePath,"wb");
	
	if (destFile==NULL)
	{
		perror("error creating file");
		printf("\n\error opening destFile file: %s\n ",destFilePath);
		printf("\n\destFolderPath: %s\t filename: %s\n ",destFolderPath,filename);
		sim_fclose(sourceFile);
		return 0;
		
	}
	
	while((bytesRead=fread(buffer,1,sizeof(buffer),sourceFile))>0){
		fwrite(buffer,1,bytesRead,destFile);
	}
	
	sim_fclose(sourceFile);
	sim_fclose(destFile);
	
	//printf("\nfile copied to: %s\n",destFilePath);
	return 1;
	

}

int CopyFile2(char *sourceFilePath,char *destFolderPath,int mod)
{
	char destFilePath[260];
	FILE *sourceFile,*destFile;
	char buffer[1024];
	size_t bytesRead;
	
	const char *filename = strrchr(sourceFilePath,'\\');
	
	if(!filename)
	{
		filename = strrchr(sourceFilePath,'/');
	}	
	if(filename)
	{
		filename ++;
	}else
	{
		filename = sourceFilePath;
	}
	

	if(mod==0)
	{
		char clearFileN[100];
		strncpy(clearFileN,filename,sizeof(clearFileN)-1);
		clearFileN[sizeof(clearFileN)-1] ='\0';
		
		sprintf(destFilePath,"%s\\%s",destFolderPath,clearFileN);
	}
	else if(mod==1)
		strcpy(destFilePath,destFolderPath);
	else
		sprintf(destFilePath,"%s\\%s",destFolderPath,filename);

	sourceFile=sim_fopen(sourceFilePath,"rb");
	
	if (sourceFile==NULL)
	{
		perror("error creating file");
		printf("\n\error opening source file: %s\n ",sourceFile);
		return 0;
	}
	
	destFile=sim_fopen(destFilePath,"wb");
	
	if (destFile==NULL)
	{
		perror("error creating file");
		printf("\n\error opening destFile file: %s\n ",destFilePath);
		//printf("\n\destFolderPath: %s\t filename: %s\n ",destFolderPath,filename);
		sim_fclose(sourceFile);
		return 0;
		
	}
	
	while((bytesRead=fread(buffer,1,sizeof(buffer),sourceFile))>0){
		fwrite(buffer,1,bytesRead,destFile);
	}
	
	sim_fclose(sourceFile);
	sim_fclose(destFile);
	
	printf("\nfile copied to: %s\n",destFilePath);
	return 1;
	

}

void input_tele(char* s,int col,int rows,float tele_out[][8])
{
	int  i, j, k;
	//SetMatZero(tele_snsr);   

	FILE* tele_file;
	tele_file = sim_fopen(s, "r");
	//skip_line(1, tele_file);
	//skip_line(2, tele_file);

	for (j = 0; j < rows; ++j)
		for (i = 0; i < col; ++i)
			fscanf(tele_file, "%f", &tele_out[j][i]);
/************************* 80/10/2 _H.GH. ************************/

	sim_fclose(tele_file);

	return;
}
void input_tele19(char* s,int col,int rows,float tele_out[][19])
{
	int  i, j, k;
	FILE* tele_file;
	tele_file = sim_fopen(s, "r");

	for (j = 0; j < rows; ++j)
		for (i = 0; i < col; ++i)
			fscanf(tele_file, "%f", &tele_out[j][i]);
/************************* 80/10/2 _H.GH. ************************/

	sim_fclose(tele_file);

	return;
}
double tele_sns_interpol(int m,double tt,float mx[][8])
{
	//tt -=tt_tele0;
	   int i,index=0;
	   
     int itele=tele_sns_sz;//18095;
	 if(tt==0.) return mx[0][m];
	   for(i=0;i<itele+1;i++) 
		if(tt<=mx[i][0]){index=i-1; break;}
		
	   if(tt==0.) index=0;
	   if(tt>tt_tele) return mx[itele-1][m];
	   else return (mx[index][m] + (mx[index+1][m]-mx[index][m]) * (tt-mx[index][0]) / (mx[index+1][0]-mx[index][0]));
}

double tele_19_interpol(int m,double tt,float mx[][19])
{
	//tt -=tt_tele0;
	   int i,index=0;
     int itele=tele_5hz_sz;//18095;
	 if(tt==0.) return mx[0][m];
	   for(i=0;i<itele+1;i++) 
		if(tt<=mx[i][0]){index=i-1; break;}
		
	   if(tt==0.) index=0;
	   if(tt>tt_tele) return mx[itele-1][m];
	   else return (mx[index][m] + (mx[index+1][m]-mx[index][m]) * (tt-mx[index][0]) / (mx[index+1][0]-mx[index][0]));
}