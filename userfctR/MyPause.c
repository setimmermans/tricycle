//---------------------------
// author: Ghijselings Pierre (source: http://faq.cprogramming.com/cgi-bin/smartfaq.cgi?id=1043284385&answer=1042856625)
// date: 21/11/16
// version 1..0
//
// Permet d'attendre que l'opérateur appuie sur une touche pour continuer
//---------------------------

//#include<stdio.h>


void mypause ( void ) 
{ 
  printf ( "Press [Enter] to continue . . ." );
//  fflush ( stdout );
  getchar();
} 
