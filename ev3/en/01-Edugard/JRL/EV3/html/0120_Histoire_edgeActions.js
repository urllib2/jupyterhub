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
         sym.stop();
         x=-1;
         
         sym.getSymbol("Symbol_1").stop(0);
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 3000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4373, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1}", "click", function(sym, e) {
         
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(1000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(2000);
         
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(3250);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(4250);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy3}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(5250);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(6250);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 6250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy4}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(7750);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(8750);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 7500, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 8750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy4}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(8750);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy5}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(9500);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(10500);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy5}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(10500);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 9250, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 12000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 10750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 11025, function(sym, e) {
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 10500, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy8}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(11000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(12000);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 12750, function(sym, e) {
         
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy9}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(13000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(14000);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy10}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(14000);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 14000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy10}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(15000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(16000);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy11}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(16000);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 14750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 16000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy11}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(18000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(19000);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy12}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(19000);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 19000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 17750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy12}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(20000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(21000);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy13}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(21000);
         
         sym.getSymbol("Symbol_1").playReverse();
         x=-1;

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 19726, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 21000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy13}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy").hide();
         sym.$("clicable1Copy14").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(22000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(23000);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy14").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy14}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(23000);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 23000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 21750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${clicable1Copy14}", "click", function(sym, e) {
         
         sym.$("clicable1").hide();
         sym.$("clicable1Copy1").hide();
         sym.$("clicable1Copy2").hide();
         sym.$("clicable1Copy3").hide();
         sym.$("clicable1Copy4").hide();
         sym.$("clicable1Copy5").hide();
         sym.$("clicable1Copy8").hide();
         sym.$("clicable1Copy9").hide();
         sym.$("clicable1Copy10").hide();
         sym.$("clicable1Copy11").hide();
         sym.$("clicable1Copy12").hide();
         sym.$("clicable1Copy13").hide();
         sym.$("clicable1Copy").hide();
         if (sym.isPlaying()) {
         
         } else if (x==-1) {
         	sym.play(24000);
         	sym.getSymbol("Symbol_1").play();
         	x=1;
         
         }
         else
         {sym.playReverse(25000);
         sym.$("clicable1").show();
         sym.$("clicable1Copy1").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy").show();
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         }
         
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy15}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(25000);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 23750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 25000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(2000);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(4250);
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy3}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(6250);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${A1_1732Copy9}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.$("clicable1").show();
         sym.$("clicable1Copy").show();
         sym.$("clicable1Copy2").show();
         sym.$("clicable1Copy3").show();
         sym.$("clicable1Copy4").show();
         sym.$("clicable1Copy5").show();
         sym.$("clicable1Copy8").show();
         sym.$("clicable1Copy9").show();
         sym.$("clicable1Copy10").show();
         sym.$("clicable1Copy11").show();
         sym.$("clicable1Copy12").show();
         sym.$("clicable1Copy13").show();
         sym.$("clicable1Copy14").show();
         sym.playReverse(12000);
         
         sym.getSymbol("Symbol_1").playReverse();
         
         x=-1;

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
   
   //Edge symbol: 'Symbol_1'
   (function(symbolName) {   
   
   })("Symbol_1");
   //Edge symbol end:'Symbol_1'

   //=========================================================
   
   //Edge symbol: 'TITRE_1'
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

   })("TITRE_1");
   //Edge symbol end:'TITRE_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");