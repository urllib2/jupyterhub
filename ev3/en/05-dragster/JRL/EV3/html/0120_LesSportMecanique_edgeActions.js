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
         sym.$("Description").html("Karting is the beginning of the career of any professional driver. Karting races take place on small tracks from 700 to 1500m, using small cars.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("karting");
         }
         else if(nb==2)
         {
         sym.getSymbol("box").play(3000);
         sym.$("Description").html("Formula 1 is a motor sport which consists of races on circuits or in the city, in aerodynamic single-seater cars, driving at very high speeds.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("The Formule1");
         }
         
         else if(nb==3)
         {
         sym.getSymbol("box").play(6000);
         sym.$("Description").html("The rally is an automobile discipline where the participants are brought to set the best time in each episode. The race takes place on dirt, on the road, and even on snow.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("The rally");
         }
         
         else if(nb==4)
         {
         sym.getSymbol("box").play(9000);
         sym.$("Description").html("Sports car racing is a form of road racing that uses two-seat, closed-wheel sports cars. They can be specially designed or linked to road models.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("The sport car race");
         
         }
         else if(nb==5)
         {
         sym.getSymbol("box").play(12000);
         sym.$("Description").html("Dragster is a mechanical acceleration sport open to two and four wheel vehicles. it is a question of putting the least possible time to cross a determined distance.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("The dragster");
         }
         else if(nb==6)
         {
         sym.getSymbol("box").play(15000);
         sym.$("Description").html("Mechanical sports are sports with motor vehicles such as cars, quads, and motorcycles,… There are many disciplines of mechanical sport.");
         sym.$("Description").delay(500).fadeIn(800);
         sym.$("Titre").delay(500).fadeIn(800);
         sym.$("Titre").html("Mecanical sports");
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