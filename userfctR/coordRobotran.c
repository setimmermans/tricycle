//---------------------------
// author: Pierre Ghijselings
// date: 16/10/16
// version 5.2
//
// Cette fonction a pour but de créer un fichier txt contenant les coordonnées des anchor points en valeur relative, afin que ce fichier soit lu par le logiciel SolidWorks. 
//---------------------------


#include "coordRobotran.h"



void anchor_points_coord()	
{
	
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                     LOADING                               *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	// copier coller de main.c de MBProject Pendulum

	MbsData *mbs_data;

    printf("Loading the Tricycle data file For second time !\n");
    mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/tricycle.mbs", BUILD_PATH);
    printf("*.mbs file loaded!\n");



    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                     INIT
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


	MbsPart *mbs_part;
	MbsDirdyn *mbs_dd;

	mbs_dd = mbs_new_dirdyn(mbs_data);
	const char* mbsfile;
	mbsfile = PROJECT_SOURCE_DIR"/../dataR/tricycle.mbs";
	MDS_gen_strct *gen;

	gen =  MDS_mbs_reader(mbsfile);

	MbsSensor *PtrSensor = mbs_dd->mbs_aux->psens;				

	int id=0;

	FILE *reading_file = NULL;
	FILE* writing_file = NULL;



    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*              COORDINATE PARTITIONING                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    mbs_part = mbs_new_part(mbs_data);
    mbs_part->options->rowperm=1;
    mbs_part->options->verbose = 1;
    mbs_run_part(mbs_part, mbs_data);
    
    mbs_delete_part(mbs_part);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                     FONCTION EQUIL
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	MbsEquil *mbs_equil;


	mbs_data->process = 2; // equil !
	mbs_equil = mbs_new_equil(mbs_data);

	// equil options (see documentations for additional options)
	mbs_equil->options->method = 1;

	mbs_equil->options->senstol = 1e-02;
	mbs_equil->options->relax = 0.6;
	mbs_equil->options->smooth = 1;
	mbs_equil->options->verbose = 0;

	printf("\n\n Run equilibrium \n");
	mbs_run_equil(mbs_equil, mbs_data);
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                     NAMES ANCHOR POINTS			 *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	// lit les noms des anchors points via le fichier all_user_id.h et écrit les noms entre guillements = la valeur de l'anchor point dans un fichier "anchor_points.txt". Les anchors points sont notés en 3 variables par le leur nom suivi de "_id" et suivi de X, Y ou Z selon que c'est exprimé selon l'un ou l'autre axe. 
	
    char chaine[1000] = "";
    int flag = 0;
    const char separation_term[2] = " ";
    char *sub_chaine;
    int anchor_number = 1;

    reading_file = fopen(PROJECT_SOURCE_DIR"/../userfctR/user_all_id.h", "r");

    if (reading_file != NULL)	

    {
        printf("Fichier user_all_id.h a bien ete ouvert\n");

	writing_file = fopen("anchor_points.txt", "w+");	

	if (writing_file != NULL)
	{
		printf("Fichier d'ecriture a bien ete ouvert\n");

		while (fgets(chaine, 1000, reading_file) != NULL) // On lit le fichier ligne par ligne tant qu'on ne reçoit pas d'erreur (NULL)
		{

			//printf("boucle \n");
		    if( flag==0 && strcmp(chaine, "// point\n") == 0 )	// Une fois arrivé à la ligne contenant "// points"
		    {

				//printf("Premier if \n");

			fseek(reading_file, 1, SEEK_CUR);		// avance le curseur d'un caractere pour passer le \n
			flag = 1;					// mnt qu'on a trouvé où se trouve les points, on mais le flag ON
		    }

		    else if ( flag == 1 && strcmp(chaine, "\n") == 0 )	// à la fin de la section "// points" on stoppe la lecture
		    {

				//printf("deuxieme if \n");
			//printf("fin de lecture des points");
			break;

		    }

		    else if ( flag == 1 ){		
			   
			// on separe la ligne en 3 parties et on prend la 2e partie ce qui nous donne le nom
			sub_chaine = strtok(chaine, separation_term);
			sub_chaine = strtok(NULL, separation_term);
			printf( "test: %s\n ", sub_chaine );

			// mnt il n'y a plus qu'à écrire le fichier .txt

			fprintf(writing_file, "\"%sX\" = %f \n", sub_chaine, mbs_data->dpt[1][anchor_number]);
			fprintf(writing_file, "\"%sY\" = %f \n", sub_chaine, mbs_data->dpt[2][anchor_number]);
			fprintf(writing_file, "\"%sZ\" = %f \n", sub_chaine, mbs_data->dpt[3][anchor_number]);	

			anchor_number++;		

		    }

        	}

		fclose(writing_file);
		printf("Fichier d'écriture a bien ete ferme\n");
	}

	fclose(reading_file);
        printf("Fichier user_all_id.h a bien ete ferme\n");
    }

    else

    {
        printf("Impossible d'ouvrir le fichier user_all_id.h\n");

    }



    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   CLOSING OPERATIONS                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	
	free_sensor(PtrSensor);
	mbs_delete_data(mbs_data);

}

