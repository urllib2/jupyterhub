/***********************
* Adobe Edge Animate Composition Actions
*
* Edit this file with caution, being careful to preserve 
* function signatures and comments starting with 'Edge' to maintain the 
* ability to interact with these actions from within Adobe Edge Animate
*
***********************/
(function($, Edge, compId){
var Composition = Edge.Composition, Symbol = Edge.Symbol; // aliases for commonly used Edge classes

   //Edge symbol: 'stage'
   (function(symbolName) {
      
      
      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         nb=0;
         nbp=0;
         sym.$("Description").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5500, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${Next}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("Description").hide();
         sym.$("Titre").hide();
         sym.$("Text").hide();
         
         nb=nb+1;
         
         if(nb==1)
         {
         sym.getSymbol("box").play(0);
         sym.$("pointer").hide();
         sym.$("Description").html("Le karting est le début de la carrière de tout pilote professionnel.Les courses de Karting se déroulent sur des petites pistes de 700 à 1500m, utilisant des petites voitures ");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("Le karting");
         }
         else if(nb==2)
         {
         sym.getSymbol("box").play(3000);
         sym.$("Description").html("La formule1 est un sport automobile qui consiste en des courses sur des circuits ou en ville, dans des voitures monoplaces aérodynamiques, roulant à des vitesses très élevées.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("La Formule1");
         }
         
         else if(nb==3)
         {
         sym.getSymbol("box").play(6000);
         sym.$("Description").html("Le rallye est une discipline automobile ou les participants sont amenées à faire le meilleur temps dans chaque épisode. La course se déroule sur la terre, sur la route, et même sur la neige.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("Le rallye");
         }
         
         else if(nb==4)
         {
         sym.getSymbol("box").play(9000);
         sym.$("Description").html("La course de voitures de sport est une forme de course automobile sur route qui utilise des voitures de sport à deux sièges et à roues fermées. Ils peuvent être spécialement conçus ou liés à des modèles routiers.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("La course d'endurance automobile");
         
         }
         else if(nb==5)
         {
         sym.getSymbol("box").play(12000);
         sym.$("Description").html("Le dragster est un sport mécanique d'accélération ouvert aux véhicules à deux et quatre roues. il s'agit de mettre le moins de temps possible pour franchir une distance determiné.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("Le dragster");
         }
         else if(nb==6)
         {
         sym.getSymbol("box").play(15000);
         sym.$("Description").html("Les sports mécaniques sont des sports avec des véhicules à moteur comme les voitures, les quads, et les motos,… ou le pilote de l’automobile,  parcourir un circuit pour établir un record, ou confronté à d’autres pilotes.Il existe de nombreuses disciplines.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("Les Sports Mécanique");
         nb=0;
         }
         
         
         

      });
      //Edge binding end

      

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

   //=========================================================
   
   //Edge symbol: 'TITRE_2_1'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         // insérer le code ici
      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5250, function(sym, e) {
         // insérer le code ic
         sym.stop();

      });
      //Edge binding end

   })("TITRE_2_1");
   //Edge symbol end:'TITRE_2_1'

   //=========================================================
   
   //Edge symbol: 'box'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 7250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 10250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 13250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 16250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

   })("box");
   //Edge symbol end:'box'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");