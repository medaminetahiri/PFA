// UNIVERSITE HASSAN II DE CASABLANCA                                                                                                                                                                                                                          ANNEE UNIVERSITAIRE : 2021-2022   //
// FACULTE DES SCIENCES ET TECHNIQUES - MOHAMMEDIA                                                                                                                                                                                                                                               //
// DEPARTEMENT GENIE ELECTRIQUE                                                                                                                                                                                                                                                                  //
// F.I GENIE ELECTRIQUE ET CONTROLE INDUSTRIEL                                                                                                                                                                                                                                                   //
// TAHIRI MOHAMED AMINE                                                                                                                                                                                                                                                                          //
//                                                                                                              PROJET DE FIN D'ANNEE :    DEPLOIMENT D'UN HOME ENERGY MANAGEMENT SYSTEM                                                                                                         //
//                                                                                                                                                                                                                                                                                               //
// R??sum??   :                                                                                                                                                                                                                                                                                    //
// Remarque??: ce code d'HEMS n??cessite un module de courant et un module de tension pour d??terminer la valeur de puissance et ESP6288-01 pour se communiquer ?? l'internet.                                                                                                                        //
// Remarque : ce code surveille la tension RMS, le courant RMS, la puissance (puissance apparente) et la puissance instantan??e (puissance active), la fr??quence, le facteur de puissance et l'??nergie accumul??e.                                                                                 //
// Remarque??: La valeur affich??e dans Serial Monitor est actualis??e toutes les secondes, peut ??tre utilis??e pour 50??Hz et 60??Hz.                                                                                                                                                                 //
// Remarque??: La fr??quence est mesur??e en comptant le temps et en moyenne pour 20????chantillons pr??lev??s (1????chantillon correspond ?? 1??cycle).                                                                                                                                                    //
// Remarque??: il n'est pas recommand?? d'ajouter un DATALOGGER d'enregistreur de donn??es pour l'enregistrement, car lenregistrement et deja sauvegard?? dans le Cloud (Thingspeak) et la pr??cision des valeurs mesur??es peut ??tre insuffisante en raison d'un probl??me de m??moire.                 //
// Remarque??: L'??talonnage automatique (tensiondoffset1 et CourantOffset1) utilise la valeur moyenne de lecture analogique de 1??000????chantillons.                                                                                                                                                 //
// Remarque : L'??talonnage automatique (OffsetTensionRMS & CourantOffset2) utilise la valeur RMS calcul??e, y compris la valeur Offset1 pour l'??talonnage.                                                                                                                                          //
// Remarque : En raison de memoire la suite des taches est realis??e grace aux plusieurs Script Matlab sur la Platforme Thingspeak.                                                                                                                                                               //
// Remarque : En raison de memoire j'ai elimin?? la partie communication serie par des commentaires                                                                                                                                                                                               //
//                                                                                                                                                                                                                                                                                               //
//                                                                                                                                                                                                                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include<LiquidCrystal.h>                                 // Charger la biblioth??que de LCD 
LiquidCrystal LCD(8,9,4,5,6,7);                           // Cr??ation de l'objet LiquidCrystal nomm?? LCD et identification des broches utilis??s pour l'affichage
#include <SoftwareSerial.h>
#define RX 2
#define TX 3
SoftwareSerial      esp8266(RX,TX); 


int                 nbrcmdreussi;
int                 TempsCommande; 
boolean             trouve = false; 
float               valcapteur = 1;
unsigned long       CMP1=0;
unsigned long       CMP2=0;
 
/* 0- General */

int                 Precisiondecimale = 1;                // Precision d??cimale pour les grandes valeurs telles que la tension, la puissance et la fr??quence...
                                                          // La precision d??cimale pour les petites valeurs telles que le courant, le facteur de puissance et l'??nergie accumul??e seront x2.
/* 1- Pour la mesure de la tension */

int                 entreetension = A1;                   // Quelle broche pour mesurer la tension.
float               ValEchantTension  = 0;                // Lire la valeur d'un ??chantillon
float               voltageLastSample  = 0;               // Pour compter le temps pour chaque ??chantillon de tension, chaque 1 milliseconde 1 ??chantillon est pr??lev??
float               Accumdechantension   = 0;               // Accumulation de lectures d'??chantillons de la tension 
float               NbrEchantTension = 0;                 // Pour compter le nombre d'??chantillons de la tension.
float               ValMoyeQuadTension ;                  // Pour calculer la valeur moyenne des quadratures de tous les ??chantillons
float               rmstension ;                          // racine carr??e de ValMoyeQuadTension

/*1.1 Determiner la tension de decalage (erreur) */
    
int                 voltageOffsetRead = 0;                // Pour changer le mode de d??calage Offset(1 et 2)
float               tensiondoffset1 = 0;                   // Pour compenser l'??cart et la pr??cision. Compensez tout faux tension lorsqu'aucun courant n'est present. 
                                                          // Le d??calage s'??talonnera automatiquement lorsque le bouton SELECT sur le module d'affichage LCD est enfonc??.
float               OffsetTensionRMS = 0;                   // pour compenser la valeur de la tension en raison d'une erreur de calcul ?? partir du RMS.
float               AccumdechantensionOffset =0;            // Accumulation de lectures d'??chantillons pour le d??calage 
float               ValMoytensionOffset = 0;              // Pour calculer la valeur moyenne de tous les ??chantillons pour l'Offset de tension.
float               DernEchantTension = 0;                // Pour compter le temps pour le present ??chantillon.
float               NbrEchantTensionOffset = 0;           // Pour compter le nombre d'??chantillons pour l'Offset. 


int                 EntreeCourant = A2;                   // Definir la broche pour mesurer la valeur actuelle de courant
float               mVpourAmp = 100;                      // J'utilise le module de courant ACS712 - 20A la valeur du mVpourAmp est declar?? dans le Datasheet = 100
float               ValEchantCourant  = 0;                // lire la valeur d'??chantillon e courant
float               DernEchantCourant  = 0;               // Pour compter le temps pour chaque ??chantillon de courant, chaque 1 milliseconde 1 ??chantillon est pr??lev??
float               Nbrdechantcourant   = 0;               // Accumulation de lectures d'??chantillons pour le courant
float               NbrEchantCourant = 0;                 // Pour compter le nombre d'??chantillons pour le courant
float               ValMoyeQuadCourant ;                  // pour calculer la valeur moyenne des quadratures de tous les ??chantillons
float               ValMoyeCourant =0 ;                   // racine carr??e de ValMoyeQuadTension
float               CourantRMSFinal ;                     // la lecture finale  RMS du courant


    
int                 LectCourantOffset = 0;                // Pour changer le mode de d??calage Offset(courantOffset 1 et 2)
float               CourantOffset1 = 0;                   // Pour compenser l'??cart du courant. Compensez tout faux courant lorsque aucun courant n'est present.
                                                          // L'Offset du courant s'??talonnera automatiquement lorsque le bouton SELECT sur le module d'affichage LCD est enfonc??.
float               CourantOffset2 = 0;                   // pour compenser la valeur de la tension en raison d'une erreur de calcul ?? partir du RMS.
float               NbrdechantcourantOffset = 0;           // Accumulation de lectures d'??chantillons pour le d??calage
float               offsetValMoyeCourant = 0;             // Pour calculer la valeur moyenne de tous les ??chantillons pour l'Offset de courant.
float               DernEchantCourantOffset = 0;          // Pour compter le temps pour le present ??chantillon de l'Offset de courant.
float               NbrEchantCourantOffset = 0;           // Pour compter le nombre d'??chantillons pour l'Offset de courant. 
 
/* 3- Mesure de puissance */
        
float               EchantCourant1 ;                      // utilis?? pour calculer le courant
float               EchantCourant2 ;                      // utilis?? pour calculer le courant
float               echantcourant3 ;                      // utilis?? pour calculer le courant
float               puissanceapp;                         // la lecture de la puissance apparente (VA)
float               puissanceactive = 0;                  // la lecture de la puissance active (W) 
float               ValEchantPuissance  = 0;              // pour lire la valeur de tension X courant actuelle d'un ??chantillon
float               DernEchantPuissance   = 0;            // Pour compter le temps pour chaque ??chantillon de courant, chaque 1 milliseconde 1 ??chantillon est pr??lev??      
float               NbrEchantPuissance  = 0;              // Pour compter le nombre d'??chantillons pour la puissnce
float               SommeEchantPuissance    = 0;          // Accumulation de lectures d'??chantillons pour la puissance       
float               factdepuissance = 0;                  // pour afficher le facteur de puissance total
bool                HP= false;                            // Heures Normal 
float               ConsomHP=0;                           
float               ConsomNor=0;
            
/*3.1 D??calage de Puissance */
    
int                 LectPuissanceOffset = 0;              // pour changer le mode pour l'offset de la puissance
float               OffsetPuissance = 0;                  // Pour compenser l'??cart du la puissance. Compensez tout fausse puissance lorsque aucun courant n'est present.
                                                          // L'Offset de la puissance s'??talonnera automatiquement lorsque le bouton SELECT sur le module d'affichage LCD est enfonc??.
float               DernEchantPuissanceOffset = 0;        // Pour compter le temps pour le present ??chantillon de l'Offset de la puissance.
float               NbrEchantPuissanceOffset = 0;         // Pour compter le nombre d'??chantillons pour l'offset de la puissnce .

/* 4 - Measure Tranche*/

unsigned char       Tranche=1;
unsigned char       Option=1;

/* 5 - Mesure quotidienne de l'??nergie*/

float               consoquotidienne   = 0;               // enregistr?? en multipliant la tension RMS et le courant RMS  par unit?? de tenpms et raz apres chaque 24h
float               consomensuelle   = 0;                 // enregistr?? en multipliant la tension RMS et le courant RMS  par unit?? de tenpms et raz apres chaque 30J
float               consomensuelleHP   = 0;       
float               DernEchantEnergie = 0;                // Utilis?? pour compter le temps pour la puissance apparente    
float               nbrechantenergie= 0;                  // Pour compter le nombre d'??chantillons.
float               energie = 0;                          // ??nergie accumul??e totale
float               EnergieAccumulee = 0;                 // difference de l'??nergie accumul??e
unsigned long       debutjour=millis(); 
        
/* 5- Frequence measurement */
        
unsigned long       EpochMicros;                          // commencer ?? compter le temps pour la fr??quence (en microsecondes)
unsigned long       PresentMicros;                        // temps actuel pour la fr??quence (en microsecondes)
int                 Nbrechfrqdes = 20;                    // Il s'agit de collecter le nombre d'??chantillons. pour eviter le chauvechement des donn??es j'ai choisi 20 (il ne faut pas choisir une valeur grande)
unsigned char       jour=1;
int                 NbrEchantFrequence = 0;               // compter le nombre d'??chantillon, 1 ??chantillon ??quivalent ?? 1 cycle
float               Frequence =0 ;                        // pour afficher la valeur de la fr??quence
float               T;                                    // utilis?? pour calculer la frequence                      
float               Trace = 9;                            // utilis?? pour ******* le chemin 
float               VAnalogique = 0;                      // Lire la valeur analogique, li?? avec la lecture "entreetension = A2"  
float               Facture=0;                            // Facture mensuelle estim??e (les redevances et les TVA sur les redevances sont pris en charge)

/* 6 - Afficheur LCD  */

unsigned long       LCDEpoch;                             // commencer ?? compter le temps pour l'??cran LCD
unsigned long       PresenttMillisLCD;                    // temps actuel pour l'affichage LCD
const unsigned long periodLCD = 1000;                     // actualiser toutes les X secondes (en secondes) dans l'affichage LCD.on prend par d??faut 1000 = 1 seconde 
unsigned char       page = 1;                             // retourner la page pour afficher les valeurs
int                 z = 0;                                // si z=1 activer le calcul de fr??quence, si z=0, activer l'affichage LCD
                                                          // besoin de s??parer l'affichage LCD avec le calcul de fr??quence peut ??tre d?? ?? la faible vitesse de m??moire de la carte qui ne peut pas faire les deux en m??me temps


void setup()                                              // code ?? ex??cuter une fois
{ 
      pinMode(0,OUTPUT);
      pinMode(1,OUTPUT);
      pinMode(11,OUTPUT);
      pinMode(12,OUTPUT);                                     
      
      Serial.begin(9600);                                 // pour afficher la lecture dans Serial Monitor en debut 9600 bauds 
      esp8266.begin(19200);
      
      LCD.begin(16,2);                                    // Dites ?? l'Arduino que notre LCD a 16 colonnes et 2 lignes
      LCD.setCursor(0,0);                                 // R??glez l'??cran LCD pour qu'il commence par le coin sup??rieur gauche de l'??cran
      LCD.print ("PFA : DEP. HEMS ");                      // Afficher "PFA : DEP. HEMS "
      LCD.setCursor(0,1);                                 // R??glez l'??cran LCD pour qu'il commence par le coin inferieur gauche de l'??cra
      LCD.print ("TAHIRI MED AMINE "); 
      commander("AT",5,"OK");                             // Ceci v??rifie si le module est correctement connect?? et son fonctionnement, le module r??pondra avec un accus?? de r??ception (OK).
      commander("AT+CWMODE=1",5,"OK");
      delay(300);
      LCD.setCursor(0,0);                                  
      LCD.print ("CONNECTION...  ");                       // Se connecter au point d'acces "wifi"
      commander("AT+CWJAP=\"wifi\",\"wifi tahiri\"",10,"OK");
      LCD.clear();
      LCD.setCursor(0,0); LCD.print ("CONNECTE");
      LCD.setCursor(0,1); LCD.print ("Restauration... ");
      //Serial.println("Restauration energie totale");
      energie = lecture("1774661","7");                  // Restaurer la valeur de l'energie en Cloud (au cas ou il y a une coupure de l'electricit??)
      //Serial.println("Restauration consoquotidienne");
      consoquotidienne = lecture("1774661","5");         // Restaurer la valeur de la consommation quotidienne
      //Serial.println("Restauration consomonsuelle");
      consomensuelle = lecture("1774661","4");           // Restaurer la valeur de la consommation mensuelle
      //Serial.println("Restauration jour");
      jour = lecture("1774661","1");
      //Serial.println("CMD RES");                       // Restaurer le jour
      //Serial.println(nbrcmdreussi);
      LCD.clear();                                       // vider l'afficheur
      if(nbrcmdreussi==0)                                // Nombre de commande reussi
      {    
          LCD.print ("Echouee");                         // la restauration est Echouee
          LCD.setCursor(0,1); 
          LCD.print ("Continuer ?");                     // Vouloir continuer ou presser RST et reprendre le cycle a nouveau
          while(1)
          {
              if  (analogRead (A0) < 800) break;         // si le boutton est enfonc?? 
          } 
          jour=1;
      }                                
      EpochMicros = micros();                            // Commencer ?? compter le temps pour la mesure de fr??quence
      LCDEpoch = millis();                               // Commencer ?? compter le temps pour LCD 
      CMP1=millis();                                    
      CMP2=millis();
}


void loop()                                              // Code ?? ex??cuter encore et encore 
{   
  Debut:                                   
      boucle();
      if ((millis()-CMP2)>=20000)
      {
            LCD.clear();
            LCD.print ("actualisation...");
            float relais1= lecture("1781252","1");
            float relais2 = lecture("1781252","2");
            if (relais1==-99.99 || relais2==-99.99)    // Definir un code d'erreur
            {
              commander("AT+CWJAP=\"wifi\",\"wifi tahiri\"",10,"OK");
              goto Debut;
              }
            Serial.println(relais1);
            digitalWrite(11,relais1);
            
            Serial.println(relais2);
            digitalWrite(12,relais2);
            if(Option==2)
            {
                  //HP=lecture("1781252","5");
                  //Serial.println(HP);
            } 
            CMP2=millis();
      }
      Serial.println(millis()-CMP1);
      if ((millis()-CMP1)>=600000)
      {     
            LCD.setCursor(0,0); LCD.print ("Envoi en cours...");
            LCD.setCursor(0,1); LCD.print ("Veillez patienter SVP");
            commander("AT",5,"OK");
            commander("AT+CWMODE=1",5,"OK");
            commander("AT+CWJAP=\"wifi\",\"wifi tahiri\"",10,"OK");
            String getData =  "GET /update?api_key=5ONSPUJ8GLDPSF07&field1="+String(jour)+"&field2="+String(CourantRMSFinal)+"&field3="+String(puissanceactive)+"&field4="+String(consomensuelle)+"&field5="+String(consoquotidienne)+"&field6="+String(Facture)+"&field7="+String(energie)+"&field8="+String(Frequence);           
            delay(300);
            envoyer(getData);
            CMP1=millis();
      }
}

float lecture(String IDCanal,String field)
{
      float valeur;
      // preparation du string GET
      String getData = "GET /channels/";
      getData += IDCanal;
      getData +="/fields/"+field+"/last";
      getData += "\r\n";
      //Serial.println("chof hna");
      Serial.println(getData);
      commander("AT+CIPMUX=1",5,"OK");
      //Serial.println("hadi");
      //Serial.println("AT+CIPSTART=0,\"TCP\",\"api.thingspeak.com\",80");
      commander("AT+CIPSTART=0,\"TCP\",\"api.thingspeak.com\",80",15,"OK");
      String cmd = "AT+CIPSEND=0," +String(getData.length());
      esp8266.println(cmd);
      //Serial.print("enviado ==> lenght cmd: ");
      Serial.println(cmd);
      Serial.print("La taille :");
      Serial.println(getData.length());
      String messageBody = "";
      
      if(esp8266.find((char *)">"))
      {
          esp8266.print(getData);
          //Serial.print("envoi de ");
          //Serial.println(getData);
          delay(500);                                               // le temps de traiter le GET, sans ce d??lai affiche occup?? ?? la prochaine commande
          while (esp8266.available()) 
          {
              String line = esp8266.readStringUntil('\n');
              if (line.length() == 1) 
              { 
                messageBody = esp8266.readStringUntil('\n');
              }
          }
      }
      else
      {
          esp8266.println("AT+CIPCLOSE");    
          Serial.println("ESP8266 CIPSEND ERROR: RESENDING");
          nbrcmdreussi=0;
      }
     
      messageBody=messageBody.substring(9,messageBody.length()-9);
      valeur=messageBody.toFloat();
      Serial.print("Valeur : ");
      Serial.println(valeur);
      if(nbrcmdreussi==0) return -99.99;
      return  valeur;
}


void envoyer(String &getData)
{
      //Serial.println("chouf hna");                            //Serial.println(getData);
      commander("AT+CIPMUX=1",5,"OK");
      commander("AT+CIPSTART=0,\"TCP\",\"api.thingspeak.com\",80",15,"OK");
      commander("AT+CIPSEND=0," +String(getData.length()+4),4,">");
      commander(getData,10,"OK");
      nbrcmdreussi++;
}
  
void boucle(){
  
 
      int buttonRead=analogRead (0);                                                     
      /*Bouton droit press?? */
      if (buttonRead < 60) 
      {   
          jour-=1;
          if (jour==0) jour=30;
          LCD.clear();
          LCD.setCursor(0,0); LCD.print ("JOUR--         \n   "); 
          LCD.setCursor(0,1); LCD.print (jour);
          delay(1500);
         
      }       

      /* Le bouton Haut est press?? */
      else if (buttonRead < 200) 
      {   
            page = 1;                                                                  // Appuyez sur le bouton haut pour aller ?? la page 1
      }    
           
      /* Le bouton Bas est press?? */
      else if (buttonRead < 400)                                                       // Appuyez sur le bouton bas pour passer ?? la page 2
      {   
            page = 2;
      }      

      /* Le bouton Gauche est press?? */
      else if (buttonRead < 600)
      {   
            jour+=1;
            if (jour==31) jour=1;
            LCD.clear();
            LCD.setCursor(0,0); LCD.print ("JOUR++         \n   "); 
            LCD.setCursor(0,1); LCD.print (jour);
            delay(1500);
      } 

      /* Le bouton Selection est press?? */
      else if (buttonRead < 800)
      {   
          //digitalWrite(11,LOW);
          //digitalWrite(12,LOW);
          LectCourantOffset = 1;                                                      // pour decaler le courant vers 0 
          voltageOffsetRead = 1;                                                      // pour decaler la tension vers 0 
          LectPuissanceOffset   = 1;                                                  // pour decaler la puissance vers 0 
          LCD.setCursor(0,0);                                                         
          LCD.print ("CALIBRATION..... ");
          LCD.setCursor(0,1);                                                        
          LCD.print ("ATTENDEZ 5 SEC ..... ");
      }

      /* 1- Mesure de tension */
  
      if(millis() >= voltageLastSample + 1 )                                          // chaque 1 milliseconde prenant 1 lecture
      {
          ValEchantTension = 2*(analogRead(entreetension)- 512) + tensiondoffset1;     // lire la valeur de l'??chantillon
          AccumdechantensionOffset = AccumdechantensionOffset + ValEchantTension;         // les valeurs s'accumulent toutes les millisecondes
          Accumdechantension = Accumdechantension + sq(ValEchantTension) ;                // accumulation d'??chantillons avec les plus anciennes    
          NbrEchantTension = NbrEchantTension + 1;                                    // incrementer le nombre des echantillons
          voltageLastSample = millis() ;                                              // affecter le temps present pour la mesure du temps 
      }
  
      if(NbrEchantTension == 1000)                                                    // apr??s 1000 comptages ou 1000 milli secondes (1 seconde), faire le calcul et afficher la valeur
      {
          ValMoytensionOffset = AccumdechantensionOffset/NbrEchantTension;             
          ValMoyeQuadTension = Accumdechantension/NbrEchantTension;                     
          rmstension = sqrt(ValMoyeQuadTension)+ OffsetTensionRMS;                      
          //Serial.print(rmstension,Precisiondecimale);
          //Serial.print(" V   ");
          Accumdechantension =0;                                                       
          NbrEchantTension=0;                                                         
          AccumdechantensionOffset=0;
      }

       /* offset de tension */
            
      if(voltageOffsetRead == 1)                                                      
      {
          tensiondoffset1 = 0; 
          if(millis()>= DernEchantTension + 1)                                        
          {                                                                            
              NbrEchantTensionOffset = NbrEchantTensionOffset + 1;                                                                        
              DernEchantTension = millis();                                                                        
          }                                                                                 
          if(NbrEchantTensionOffset == 2000)                                         
          {                                                                            
              tensiondoffset1 = -1*(ValMoytensionOffset);                             
              voltageOffsetRead = 2;                                                            
              NbrEchantTensionOffset = 0;                                                                                                     
          } 
      }   
              
      if(voltageOffsetRead == 2)                                                     
      {
          OffsetTensionRMS = 0;                                                      
          if(millis()>= DernEchantTension + 1)                                        
          {                                                                            
              NbrEchantTensionOffset = NbrEchantTensionOffset + 1;                                                                          
              DernEchantTension = millis();                                                                          
          }                                                                                
          if(NbrEchantTensionOffset == 2000)                                       
          {                                                                            
              OffsetTensionRMS = - rmstension;                                         
              voltageOffsetRead = 0;                                                                 
              NbrEchantTensionOffset = 0;                                            
          }                                                                             
      } 


  
      if(millis() >= DernEchantCourant + 1)                                          
      {       
          ValEchantCourant = analogRead(EntreeCourant)-512 + CourantOffset1;         
          NbrdechantcourantOffset = NbrdechantcourantOffset + ValEchantCourant;        
          Nbrdechantcourant = Nbrdechantcourant + sq(ValEchantCourant) ;                
          NbrEchantCourant = NbrEchantCourant + 1;                                   
          DernEchantCourant = millis();                                             
      }
  
      if(NbrEchantCourant == 1000)                                                    
      {
          offsetValMoyeCourant = NbrdechantcourantOffset/NbrEchantCourant;         
          ValMoyeQuadCourant = Nbrdechantcourant/NbrEchantCourant;               
          ValMoyeCourant = sqrt(ValMoyeQuadCourant)+CourantOffset2 ;                 
          
          CourantRMSFinal = (((ValMoyeCourant /1023) *5000) /mVpourAmp);       
          Serial.print(CourantRMSFinal,Precisiondecimale*2);
          //Serial.print(" A   ");
          Nbrdechantcourant =0;                                                       
          NbrEchantCourant=0;                                                    
          NbrdechantcourantOffset=0;                                             
      }

        /* Offset de courant */
            
      if(LectCourantOffset == 1)                                                 
      {
          CourantOffset1 = 0;                                                 
          if(millis()>= DernEchantCourantOffset + 1)                               
          {                                                                            
              NbrEchantCourantOffset = NbrEchantCourantOffset + 1;                                                                          
              DernEchantCourantOffset = millis();                                                                          
          }                                                                                 
          if(NbrEchantCourantOffset == 2000)                                 
          {                                                                            
              CourantOffset1 = - offsetValMoyeCourant;                    
              LectCourantOffset = 2;                                                  
              NbrEchantCourantOffset = 0;                                                                                              
          } 
      }   
          
      if(LectCourantOffset == 2)                                            
      {
            CourantOffset2 = 0;                                                  
            if(millis()>= DernEchantCourantOffset + 1)                             
            {                                                                            
                NbrEchantCourantOffset = NbrEchantCourantOffset + 1;                                                                          
                DernEchantCourantOffset = millis();                                                                          
            }                                                                             
                      
            if(NbrEchantCourantOffset == 2000)                                   
            {                                                                            
                CourantOffset2 = - ValMoyeCourant;                                  
                LectCourantOffset = 0;                                                                 
                NbrEchantCourantOffset = 0;                                       
            }                                                                             
      } 

 

      if(millis() >= DernEchantPuissance + 1)                                        
      {
            EchantCourant1 = analogRead(EntreeCourant)-512+ CourantOffset1;          
            EchantCourant2 = (EchantCourant1/1024)*5000;
            echantcourant3 = EchantCourant2/mVpourAmp;
            ValEchantTension = 2*(analogRead(entreetension)- 512)+ tensiondoffset1 ;   
            ValEchantPuissance = ValEchantTension * echantcourant3 ;                
            SommeEchantPuissance = SommeEchantPuissance + ValEchantPuissance ;  
            NbrEchantPuissance = NbrEchantPuissance + 1;                            
            DernEchantPuissance = millis();                                        
      }
  
      if(NbrEchantPuissance == 1000)                                                  
      {
            
            puissanceactive = ((SommeEchantPuissance/NbrEchantPuissance)+ OffsetPuissance)/1000 ;      
            //Serial.print(puissanceactive,Precisiondecimale);
            //Serial.print(" KW   ");
            puissanceapp= CourantRMSFinal*rmstension/1000  ;                       
            //Serial.print(puissanceapp,Precisiondecimale);
            //Serial.print(" KVA   ");
            factdepuissance = puissanceactive/puissanceapp;    
            if(factdepuissance >1 || factdepuissance<0)                            
            {
              factdepuissance = 0;
            }                             
            SommeEchantPuissance =0;                                                    
            NbrEchantPuissance=0;                                             
      }



      if(LectPuissanceOffset == 1)                                              
      {                          
            OffsetPuissance = 0;                                                        
            if(millis()>= DernEchantPuissanceOffset + 1)                           
            {                                                                            
                  NbrEchantPuissanceOffset = NbrEchantPuissanceOffset + 1;                                                                          
                  DernEchantPuissanceOffset = millis();                                                                          
            }                                                                                                      
            if(NbrEchantPuissanceOffset == 5000)                                    
            {                                                                            
                  OffsetPuissance = - puissanceactive;                                                
                  LectPuissanceOffset = 0;                                                           
                  NbrEchantPuissanceOffset = 0;                                    
            }                                                                             
     } 



  
      if(millis() >= DernEchantEnergie + 1)                                          
      {
            nbrechantenergie = nbrechantenergie + 1;  
            DernEchantEnergie = millis(); 
      }
     
      if(nbrechantenergie == 1000)                                                
      {
            Trace = 9; 
            z=0;
            NbrEchantFrequence = 0;
            EnergieAccumulee = puissanceactive/3600;                               
            energie = energie + EnergieAccumulee;
            consoquotidienne  = consoquotidienne+ EnergieAccumulee;
            consomensuelle=consomensuelle+EnergieAccumulee;
            if(Option==2 && HP==false)
            {
              //Serial.print(" False  ");
              ConsomNor = ConsomNor + EnergieAccumulee;  
            }
            if(Option==2 && HP==true)
            {
             //Serial.print(" True ");
             ConsomHP=ConsomHP + EnergieAccumulee; 
            }
            //Serial.print(" jour  ");
            //Serial.println(jour);
            //Serial.print(" w9t  ");
            //Serial.println((millis()-CMP1));
            //if((millis()-debutjour) > 86400000) 
            if((millis()-debutjour) > 86400000)
            {
                  jour+=1;
                  debutjour=millis();
                  consoquotidienne=0;
                  if  (jour==30)
                  {
                        consomensuelle=0;
                        ConsomHP=0;
                        ConsomNor=0; 
                        jour=1;
                  }
            }
            if(Option==1)
            {
                  if(consomensuelle<101)
                  {
                        Tranche=1;
                        Facture= consomensuelle*0.9010; 
                  }
                  else if(consomensuelle>=101 && consomensuelle<151)
                  {
                        Tranche=2;
                        Facture= 100*0.9010+(consomensuelle-100)*1.0732 ;
                  }
                  else if(consomensuelle>=151 && consomensuelle<201)
                  {
                        Tranche=3;
                        Facture= consomensuelle*1.0732  ;
                  }
                  else if(consomensuelle>=201 && consomensuelle<301)
                  {
                        Tranche=4;
                        Facture= consomensuelle*1.1676 ;
                  }
                  else if(consomensuelle>=301 && consomensuelle<501)
                  {
                        Tranche=5;
                        Facture= consomensuelle*1.3817 ; 
                  }
                  else if(consomensuelle>=501)
                  {
                        Tranche=6;
                        Facture= consomensuelle*1.5958 ;             
                  }
            
                  //Serial.print("Tranche :   ");
                  //Serial.println(Tranche);
                  //Serial.print("Facture :   ");
                  //Serial.print(Facture);   
                  //Serial.println("DH   ");      
            }
            else if(Option==2)
            {
                  
                  Facture= ConsomNor*1.2467+ConsomHP*2.2441;
                  //Serial.print("Facture :   ");
                  //Serial.print(Facture);   
                  //Serial.println("DH   ");      
            }
            Facture=Facture+17.42;                                                   // Redevances Fixes
            Facture=Facture+Facture*0.0025;                                          // Frais Timbre fiscal 
            //Serial.print(consoquotidienne,Precisiondecimale);
            //Serial.println(" KWh   ");
            //Serial.print(energie,Precisiondecimale);
            //Serial.println(" KWh   ");
            
            nbrechantenergie = 0 ;                                                   
      }


      if(z == 1)                                                                   
      {
            VAnalogique = ValEchantTension;                                       
            if(VAnalogique < 0 && Trace == 9)                                  
            {
                  Trace = 8;
            }
            
            if(VAnalogique >= 0 && Trace ==8)                                  
            {
                  EpochMicros = micros(); 
                  Trace = 7;
            }
           
            if(VAnalogique < 0 && Trace == 7)                                    
            {
                    Trace = 6;
            }  
             
            if(VAnalogique >=0 && Trace == 6)                                   
            {
                   PresentMicros = micros();                                       
                   NbrEchantFrequence = NbrEchantFrequence +1 ;                     
                   Trace = 7;                                                  
            }
  
            if(NbrEchantFrequence == Nbrechfrqdes && Trace == 7 && rmstension> 100) 
            {
                     
                    T = PresentMicros-EpochMicros ;                                
                    Frequence = 1/((T/1000000)/NbrEchantFrequence);            
                    //Serial.print(Frequence,Precisiondecimale);                     
                    //Serial.println(" Hz  ");
                    NbrEchantFrequence = 0;                                      
                    Trace = 9;                                                   
                    z = 0;                                                    
            }
      }
  

  /* affichage LCD   */
  
      PresenttMillisLCD = millis();                                                
      if (PresenttMillisLCD - LCDEpoch >= periodLCD && page ==1 && Option==1)   
      {
            LCD.setCursor(0,0);                                                    
            LCD.print(CourantRMSFinal,Precisiondecimale*2);                
            LCD.print("A    ");
            LCD.setCursor(8,0);
            LCD.print(Facture,Precisiondecimale);                              
            LCD.print("DH    ");   
            LCD.setCursor(0,1); 
            LCD.print(puissanceactive,Precisiondecimale);
            LCD.print("KW     ");
            LCD.setCursor(8,1);
            LCD.print(consomensuelle,Precisiondecimale);                          
            LCD.print("KWh ");
            LCDEpoch = PresenttMillisLCD ;                                    
            z = 1;                                                           
      }


      if (PresenttMillisLCD - LCDEpoch >= periodLCD && page ==1 && Option==2)     
      {
            LCD.setCursor(0,0);                                                   
            LCD.print(puissanceactive,Precisiondecimale*2);            
            LCD.print("KW     ");
            LCD.setCursor(8,0);
            LCD.print(Facture,Precisiondecimale);                          
            LCD.print("DH    ");   
            LCD.setCursor(0,1); 
            LCD.print(ConsomNor,Precisiondecimale);
            LCD.print("KWh ");
            LCD.setCursor(8,1);
            LCD.print(ConsomHP,Precisiondecimale);                              
            LCD.print("KWh ");
            LCDEpoch = PresenttMillisLCD ;                                          
            z = 1;                                                              
      }
      
      if( PresenttMillisLCD - LCDEpoch >= periodLCD && page ==2)               
      {
            LCD.setCursor(0,0);                                                    
            LCD.print(consoquotidienne,Precisiondecimale);
            LCD.print("KWh     "); 
            LCD.setCursor(9,0);
            LCD.print(Frequence,Precisiondecimale*2);
            LCD.print("Hz          ");
            LCD.setCursor(0,1); 
            LCD.print(energie,Precisiondecimale*2); 
            LCD.print("kWh                     "); 
            LCDEpoch = PresenttMillisLCD ;                                        
            z = 1;                                                               
      }
   
}

void commander(String command, int TempsMax, char LireReponse[]) 
{
      //Serial.print(nbrcmdreussi);
      //Serial.print(". at command => ");
      //Serial.print(command);
      //Serial.print(" ");
      while(TempsCommande < (TempsMax*1))
      {
            esp8266.println(command);//at+cipsend
            if(esp8266.find(LireReponse))//ok
            {
                  trouve = true;
                  break;
            }
          
            TempsCommande++;
      }
      
      if(trouve == true)
      {
            Serial.println("Succ??");
            nbrcmdreussi++;
            TempsCommande = 0;
      }
      
      if(trouve == false)
      {
            Serial.println("Fail");
            nbrcmdreussi = 0;
            TempsCommande = 0;
      }
      
      trouve = false;
}
