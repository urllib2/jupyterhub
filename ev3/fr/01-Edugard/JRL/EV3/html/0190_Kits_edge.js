/*jslint */
/*global AdobeEdge: false, window: false, document: false, console:false, alert: false */
(function (compId) {

    "use strict";
    var im='images/',
        aud='media/',
        vid='media/',
        js='js/',
        fonts = {
            'Digital-7': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            '\'Digital tech\'': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            'MagistralC': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />'        },
        opts = {
            'gAudioPreloadPreference': 'auto',
            'gVideoPreloadPreference': 'auto'
        },
        resources = [
        ],
        scripts = [
            js+"jquery-2.0.3.min.js",
            js+"jquery.ui.touch-punch.min.js",
            js+"jquery-ui.min.js"
        ],
        symbols = {
            "stage": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "height",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['85px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['555px', '29', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'Imgs',
                            type: 'group',
                            rect: ['-25px', '-48px', '1440', '1080px', 'auto', 'auto'],
                            clip: 'rect(151.8519287109375px 1507.87451171875px 221.666748046875px 0px)',
                            c: [
                            {
                                id: 'PRO1-33',
                                type: 'image',
                                rect: ['295px', '156px', '1080px', '45px', 'auto', 'auto'],
                                clip: 'rect(0px 1232.77783203125px 45px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"A1_tabs_01.png",'0px','0px'],
                                transform: [[],[],[],['1.28704','1.04444']]
                            },
                            {
                                id: 'PRO1-3',
                                type: 'image',
                                rect: ['140px', '155px', '1390px', '47px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_tabs_03.png",'0px','0px']
                            },
                            {
                                id: 'PRO2-3',
                                type: 'image',
                                rect: ['140px', '155px', '1390px', '47px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_tabs_04.png",'0px','0px']
                            },
                            {
                                id: 'PRO3-3',
                                type: 'image',
                                rect: ['140px', '155px', '1390px', '47px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_tabs_02.png",'0px','0px']
                            },
                            {
                                id: 'PRO4-3',
                                type: 'image',
                                rect: ['140px', '155px', '1390px', '47px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_tabs_05.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'brick',
                            type: 'group',
                            rect: ['252px', '-47px', '1116px', '937px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'brique',
                                type: 'image',
                                rect: ['292px', '462px', '632px', '421px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"A1_briqueIn.png",'0px','0px'],
                                transform: [[],['31'],[],['1.14999','1.14999']]
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['156px', '245px', '831px', '152px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['189px', '268px', '768px', '103px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-weight: 500; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 34px;\">La brique intelligente est le cerveau de robot.</span></p><p style=\"margin: 0px;\"><span style=\"font-weight: 500; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 34px;\">Elle permet d'excuter tous vos programmes.</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'acceuill2',
                            type: 'group',
                            rect: ['288px', '81px', '1179px', '846', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_YELLOWCopy',
                                type: 'image',
                                rect: ['43px', '119px', '985px', '178px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Text2Copy2',
                                type: 'text',
                                rect: ['86px', '22px', '950px', '263px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 34px;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 400;\">​Les moteurs électriques sont couramment utilisés pour actionner divers éléments dans vos robots : ses roues, les jambes, les pistes, les bras, les doigts,...</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">​</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'moteur_moyen',
                                type: 'image',
                                rect: ['701px', '533px', '335px', '225px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"A1_moteur%20moyen.png",'0px','0px']
                            },
                            {
                                id: 'moteur_large',
                                type: 'image',
                                rect: ['67px', '519px', '322px', '250px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"A2_moteur%20large.png",'0px','0px']
                            },
                            {
                                id: 'titrewgCopy',
                                type: 'text',
                                rect: ['78px', '455px', '411px', '42px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">Le moteur large&nbsp;</p>",
                                align: "left",
                                font: ['Lucida Console, Monaco, monospace', [36, "px"], "rgba(127,79,0,1.00)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titrewgCopy2',
                                type: 'text',
                                rect: ['650px', '455px', '411px', '42px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">Le moteur moyen</p>",
                                align: "left",
                                font: ['Lucida Console, Monaco, monospace', [36, "px"], "rgba(127,79,0,1.00)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'technique',
                            type: 'group',
                            rect: ['278px', '247px', '1252', '641', 'auto', 'auto'],
                            c: [
                            {
                                id: 'legoTech',
                                type: 'image',
                                rect: ['674px', '-22px', '490px', '244px', 'auto', 'auto'],
                                opacity: '0.47',
                                fill: ["rgba(0,0,0,0)",im+"A1_legoTech.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_YELLOWCopy2',
                                type: 'image',
                                rect: ['-98px', '6px', '728px', '154px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'Text5Copy',
                                type: 'text',
                                rect: ['-65px', '-12px', '657px', '165px', 'auto', 'auto'],
                                opacity: '0.47',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 34px;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">Les pièces techniques sont utilisables pour la construction d'un robot.&nbsp;​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">​</span></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'axle',
                                type: 'image',
                                rect: ['616px', '343px', '190px', '251px', 'auto', 'auto'],
                                opacity: '1',
                                fill: ["rgba(0,0,0,0)",im+"A1_axle.png",'0px','0px']
                            },
                            {
                                id: 'beams',
                                type: 'image',
                                rect: ['243px', '355px', '274px', '250px', 'auto', 'auto'],
                                opacity: '0.47',
                                fill: ["rgba(0,0,0,0)",im+"A1_beams.png",'0px','0px']
                            },
                            {
                                id: 'gears',
                                type: 'image',
                                rect: ['-79px', '343px', '204px', '260px', 'auto', 'auto'],
                                opacity: '0.47',
                                fill: ["rgba(0,0,0,0)",im+"A1_gears.png",'0px','0px']
                            },
                            {
                                id: 'wheels',
                                type: 'image',
                                rect: ['883px', '380px', '274px', '195px', 'auto', 'auto'],
                                opacity: '0.47',
                                fill: ["rgba(0,0,0,0)",im+"A1_wheels.png",'0px','0px']
                            },
                            {
                                id: 'titrew',
                                type: 'text',
                                rect: ['898px', '288px', '209px', '42px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">Les roues</p>",
                                align: "left",
                                font: ['Lucida Console, Monaco, monospace', [36, "px"], "rgba(127,79,0,1.00)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titrewa',
                                type: 'text',
                                rect: ['607px', '288px', '209px', '42px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">Les axes</p>",
                                align: "left",
                                font: ['Lucida Console, Monaco, monospace', [36, "px"], "rgba(127,79,0,1.00)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titrewb',
                                type: 'text',
                                rect: ['255px', '289px', '269px', '42px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">Les poutres</p>",
                                align: "left",
                                font: ['Lucida Console, Monaco, monospace', [36, "px"], "rgba(127,79,0,1.00)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titrewg',
                                type: 'text',
                                rect: ['-142px', '289px', '335px', '42px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">Les engrenages</p>",
                                align: "left",
                                font: ['Lucida Console, Monaco, monospace', [36, "px"], "rgba(127,79,0,1.00)", "800", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'capteurs',
                            type: 'group',
                            rect: ['265px', '81px', '1197px', '846', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_YELLOWCopy5',
                                type: 'image',
                                rect: ['26px', '148px', '1024px', '227px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'captGYro',
                                type: 'image',
                                rect: ['288px', '553px', '246px', '185px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"captGYro.png",'0px','0px']
                            },
                            {
                                id: 'captTac',
                                type: 'image',
                                rect: ['549px', '553px', '246px', '185px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"captTac.png",'0px','0px']
                            },
                            {
                                id: 'capUltra2',
                                type: 'image',
                                rect: ['816px', '553px', '246px', '185px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"capUltra.png",'0px','0px']
                            },
                            {
                                id: 'capCoul',
                                type: 'image',
                                rect: ['14px', '553px', '247px', '185px', 'auto', 'auto'],
                                cursor: 'pointer',
                                opacity: '1',
                                fill: ["rgba(0,0,0,0)",im+"capCoul.png",'0px','0px']
                            },
                            {
                                id: 'Text2Copy5',
                                type: 'text',
                                rect: ['56px', '173px', '978px', '265px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">Les capteurs permettent au robot de recevoir des informations sur son environnement.</span></p><p style=\"margin: 0px;\">Ils sont essentiels au fonctionnement d'un robot car il a besoin d'informations sur son milieu extérieur&nbsp;</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Button1_Text',
                            type: 'text',
                            rect: ['122px', '112px', '349px', '40', 'auto', 'auto'],
                            clip: 'rect(0px 343.35498046875px 40px 15.068359375px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\">&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;Brique Intelligente &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;&nbsp;</p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'Button1_Text1Copy',
                            type: 'text',
                            rect: ['122px', '112px', '358px', '40', 'auto', 'auto'],
                            clip: 'rect(0px 367.91650390625px 40px 83.4892578125px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\"><span style=\"color: rgb(255, 242, 221);\">&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;Brique Intelligente &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;&nbsp;</span></p>",
                            align: "left",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'Button2_Text',
                            type: 'text',
                            rect: ['499px', '112px', '300', '40', 'auto', 'auto'],
                            clip: 'rect(0px 316.438232421875px 40px 20.5478515625px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\">​Moteurs</p>",
                            align: "center",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'Button2_Text1Copy',
                            type: 'text',
                            rect: ['499px', '112px', '300', '40', 'auto', 'auto'],
                            clip: 'rect(0px 316.438232421875px 40px 20.5478515625px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\">​<span style=\"color: rgb(255, 242, 221);\">Moteurs</span></p>",
                            align: "center",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'Button3_Text',
                            type: 'text',
                            rect: ['816px', '112px', '300', '40', 'auto', 'auto'],
                            clip: 'rect(0px 310.958740234375px 40px 13.69873046875px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\">​Capteurs</p>",
                            align: "center",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'Button3_Text1Copy',
                            type: 'text',
                            rect: ['816px', '112px', '300', '40', 'auto', 'auto'],
                            clip: 'rect(0px 310.958740234375px 40px 13.69873046875px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\">​<span style=\"color: rgb(255, 242, 221);\">Capteurs</span></p>",
                            align: "center",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'Button4_Text',
                            type: 'text',
                            rect: ['1131px', '112px', '300', '40', 'auto', 'auto'],
                            clip: 'rect(0px 310.958740234375px 40px 13.69873046875px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\">​Pieces techniques</p>",
                            align: "center",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'Button4_Text1Copy',
                            type: 'text',
                            rect: ['1131px', '112px', '300', '40', 'auto', 'auto'],
                            clip: 'rect(0px 310.958740234375px 40px 13.69873046875px)',
                            cursor: 'pointer',
                            text: "<p style=\"margin:0px\">​<span style=\"color: rgb(255, 242, 221);\">Pieces techniques</span></p>",
                            align: "center",
                            font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [26, "px"], "rgba(0,0,0,1.00)", "900", "none", "normal", "break-word", ""],
                            textStyle: ["", "", "40px", "", ""]
                        },
                        {
                            id: 'acceuill',
                            type: 'group',
                            rect: ['423px', '169px', '809px', '816px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'legoset',
                                type: 'image',
                                rect: ['100px', '-35px', '765px', '765px', 'auto', 'auto'],
                                clip: 'rect(54.54541015625px 765px 692.272705078125px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"A1_legoset.png",'0px','0px'],
                                transform: [[],[],[],['0.82235','0.82235']]
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN1',
                                type: 'image',
                                rect: ['-282px', '16px', '588px', '236px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'acceuil',
                                type: 'text',
                                rect: ['-256px', '36px', '628px', '57px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-weight: 500; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 32px;\">Le kit robotique qu'on va utiliser</span></p><p style=\"margin: 0px;\"><span style=\"font-weight: 500; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 32px;\">contient une grande variété &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; d’éléments permettant la réalisation</span></p><p style=\"margin: 0px;\"><span style=\"font-weight: 500; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 32px;\"> de projets robotiques complexes et </span></p><p style=\"margin: 0px;\"><span style=\"font-weight: 500; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 32px;\">variés.</span></p>",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "900", "none", "", "break-word", "normal"],
                                textStyle: ["", "", "40px", "", ""]
                            },
                            {
                                id: 'A1_kit2',
                                type: 'image',
                                rect: ['703px', '-6px', '390px', '472px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_kit2.png",'0px','0px']
                            },
                            {
                                id: 'A1_kit1',
                                type: 'image',
                                rect: ['-338px', '306px', '537px', '460px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A1_kit1.png",'0px','0px']
                            }]
                        }
                    ],
                    style: {
                        '${Stage}': {
                            isStage: true,
                            rect: ['null', 'null', '1650px', '1080', 'auto', 'auto'],
                            overflow: 'hidden',
                            fill: ["rgba(240,240,240,0)"]
                        }
                    }
                },
                timeline: {
                    duration: 25500,
                    autoPlay: false,
                    data: [
                        [
                            "eid599",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Text5Copy}",
                            '-12px',
                            '-12px'
                        ],
                        [
                            "eid659",
                            "scaleY",
                            19500,
                            500,
                            "linear",
                            "${captGYro}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid660",
                            "scaleY",
                            20000,
                            500,
                            "linear",
                            "${captGYro}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid647",
                            "scaleY",
                            16500,
                            500,
                            "linear",
                            "${capUltra2}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid648",
                            "scaleY",
                            17000,
                            500,
                            "linear",
                            "${capUltra2}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid713",
                            "scaleY",
                            23000,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid714",
                            "scaleY",
                            23500,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid725",
                            "opacity",
                            21000,
                            500,
                            "linear",
                            "${Text2Copy2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid726",
                            "opacity",
                            21500,
                            500,
                            "linear",
                            "${Text2Copy2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid727",
                            "opacity",
                            23000,
                            500,
                            "linear",
                            "${Text2Copy2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid728",
                            "opacity",
                            23500,
                            500,
                            "linear",
                            "${Text2Copy2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid472",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${legoTech}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid473",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${legoTech}",
                            '0.47',
                            '1'
                        ],
                        [
                            "eid491",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${legoTech}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid492",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${legoTech}",
                            '0.47',
                            '1'
                        ],
                        [
                            "eid421",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${legoTech}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid431",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${legoTech}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid451",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${legoTech}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid461",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${legoTech}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid627",
                            "scaleY",
                            15000,
                            500,
                            "linear",
                            "${capCoul}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid626",
                            "scaleY",
                            15500,
                            500,
                            "linear",
                            "${capCoul}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid542",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${titrewa}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid550",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${titrewa}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid575",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${titrewa}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid581",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${titrewa}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid589",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${titrewa}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid595",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${titrewa}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid604",
                            "left",
                            0,
                            0,
                            "linear",
                            "${titrewa}",
                            '607px',
                            '607px'
                        ],
                        [
                            "eid380",
                            "top",
                            12000,
                            500,
                            "linear",
                            "${beams}",
                            '355px',
                            '267px'
                        ],
                        [
                            "eid381",
                            "top",
                            12500,
                            500,
                            "linear",
                            "${beams}",
                            '267px',
                            '355px'
                        ],
                        [
                            "eid661",
                            "top",
                            19500,
                            500,
                            "linear",
                            "${captGYro}",
                            '553px',
                            '500px'
                        ],
                        [
                            "eid662",
                            "top",
                            20020,
                            480,
                            "linear",
                            "${captGYro}",
                            '500px',
                            '553px'
                        ],
                        [
                            "eid711",
                            "top",
                            23000,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '545px',
                            '469px'
                        ],
                        [
                            "eid712",
                            "top",
                            23500,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '469px',
                            '533px'
                        ],
                        [
                            "eid436",
                            "top",
                            13250,
                            500,
                            "linear",
                            "${gears}",
                            '343px',
                            '278px'
                        ],
                        [
                            "eid437",
                            "top",
                            13750,
                            500,
                            "linear",
                            "${gears}",
                            '278px',
                            '343px'
                        ],
                        [
                            "eid607",
                            "top",
                            0,
                            0,
                            "linear",
                            "${titrew}",
                            '288px',
                            '288px'
                        ],
                        [
                            "eid655",
                            "top",
                            18000,
                            500,
                            "linear",
                            "${captTac}",
                            '553px',
                            '500px'
                        ],
                        [
                            "eid656",
                            "top",
                            18520,
                            480,
                            "linear",
                            "${captTac}",
                            '500px',
                            '553px'
                        ],
                        [
                            "eid598",
                            "top",
                            0,
                            0,
                            "linear",
                            "${legoTech}",
                            '-22px',
                            '-22px'
                        ],
                        [
                            "eid610",
                            "left",
                            0,
                            0,
                            "linear",
                            "${axle}",
                            '616px',
                            '616px'
                        ],
                        [
                            "eid640",
                            "top",
                            15000,
                            500,
                            "linear",
                            "${capCoul}",
                            '553px',
                            '500px'
                        ],
                        [
                            "eid644",
                            "top",
                            15520,
                            480,
                            "linear",
                            "${capCoul}",
                            '500px',
                            '553px'
                        ],
                        [
                            "eid338",
                            "top",
                            9500,
                            500,
                            "linear",
                            "${wheels}",
                            '380px',
                            '323px'
                        ],
                        [
                            "eid339",
                            "top",
                            10000,
                            500,
                            "linear",
                            "${wheels}",
                            '323px',
                            '392px'
                        ],
                        [
                            "eid741",
                            "scaleY",
                            24500,
                            500,
                            "linear",
                            "${brique}",
                            '1.14999',
                            '1.34'
                        ],
                        [
                            "eid742",
                            "scaleY",
                            25000,
                            500,
                            "linear",
                            "${brique}",
                            '1.34',
                            '1.14999'
                        ],
                        [
                            "eid597",
                            "left",
                            0,
                            0,
                            "linear",
                            "${legoTech}",
                            '674px',
                            '674px'
                        ],
                        [
                            "eid432",
                            "scaleX",
                            13250,
                            500,
                            "linear",
                            "${gears}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid433",
                            "scaleX",
                            13750,
                            500,
                            "linear",
                            "${gears}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid645",
                            "scaleX",
                            16500,
                            500,
                            "linear",
                            "${capUltra2}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid646",
                            "scaleX",
                            17000,
                            500,
                            "linear",
                            "${capUltra2}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid721",
                            "opacity",
                            21000,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid722",
                            "opacity",
                            21500,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid723",
                            "opacity",
                            23000,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid724",
                            "opacity",
                            23500,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid608",
                            "left",
                            0,
                            0,
                            "linear",
                            "${beams}",
                            '243px',
                            '243px'
                        ],
                        [
                            "eid653",
                            "scaleY",
                            18000,
                            500,
                            "linear",
                            "${captTac}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid654",
                            "scaleY",
                            18500,
                            500,
                            "linear",
                            "${captTac}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid470",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${axle}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid471",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${axle}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid419",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${axle}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid429",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${axle}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid450",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${axle}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid460",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${axle}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid336",
                            "scaleY",
                            9500,
                            500,
                            "linear",
                            "${wheels}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid337",
                            "scaleY",
                            10000,
                            500,
                            "linear",
                            "${wheels}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid345",
                            "scaleX",
                            10750,
                            500,
                            "linear",
                            "${axle}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid346",
                            "scaleX",
                            11250,
                            500,
                            "linear",
                            "${axle}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid347",
                            "scaleY",
                            10750,
                            500,
                            "linear",
                            "${axle}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid348",
                            "scaleY",
                            11250,
                            500,
                            "linear",
                            "${axle}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid536",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${titrewb}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid546",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${titrewb}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid558",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${titrewb}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid564",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${titrewb}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid590",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${titrewb}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid596",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${titrewb}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid605",
                            "top",
                            0,
                            0,
                            "linear",
                            "${titrewa}",
                            '288px',
                            '288px'
                        ],
                        [
                            "eid672",
                            "opacity",
                            15000,
                            500,
                            "linear",
                            "${capUltra2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid673",
                            "opacity",
                            15500,
                            500,
                            "linear",
                            "${capUltra2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid690",
                            "opacity",
                            18000,
                            500,
                            "linear",
                            "${capUltra2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid691",
                            "opacity",
                            18500,
                            500,
                            "linear",
                            "${capUltra2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid670",
                            "opacity",
                            19500,
                            500,
                            "linear",
                            "${capUltra2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid671",
                            "opacity",
                            20000,
                            500,
                            "linear",
                            "${capUltra2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid657",
                            "scaleX",
                            19500,
                            500,
                            "linear",
                            "${captGYro}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid658",
                            "scaleX",
                            20000,
                            500,
                            "linear",
                            "${captGYro}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid678",
                            "opacity",
                            15000,
                            500,
                            "linear",
                            "${captGYro}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid679",
                            "opacity",
                            15500,
                            500,
                            "linear",
                            "${captGYro}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid680",
                            "opacity",
                            16500,
                            500,
                            "linear",
                            "${captGYro}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid681",
                            "opacity",
                            17000,
                            500,
                            "linear",
                            "${captGYro}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid692",
                            "opacity",
                            18000,
                            500,
                            "linear",
                            "${captGYro}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid693",
                            "opacity",
                            18500,
                            500,
                            "linear",
                            "${captGYro}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid707",
                            "scaleY",
                            21000,
                            500,
                            "linear",
                            "${moteur_large}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid708",
                            "scaleY",
                            21500,
                            500,
                            "linear",
                            "${moteur_large}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid719",
                            "opacity",
                            23000,
                            500,
                            "linear",
                            "${moteur_large}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid720",
                            "opacity",
                            23500,
                            500,
                            "linear",
                            "${moteur_large}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid434",
                            "scaleY",
                            13250,
                            500,
                            "linear",
                            "${gears}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid435",
                            "scaleY",
                            13750,
                            500,
                            "linear",
                            "${gears}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid603",
                            "top",
                            0,
                            0,
                            "linear",
                            "${titrewb}",
                            '289px',
                            '289px'
                        ],
                        [
                            "eid651",
                            "scaleX",
                            18000,
                            500,
                            "linear",
                            "${captTac}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid652",
                            "scaleX",
                            18500,
                            500,
                            "linear",
                            "${captTac}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid709",
                            "top",
                            21000,
                            500,
                            "linear",
                            "${moteur_large}",
                            '519px',
                            '443px'
                        ],
                        [
                            "eid710",
                            "top",
                            21500,
                            500,
                            "linear",
                            "${moteur_large}",
                            '443px',
                            '519px'
                        ],
                        [
                            "eid349",
                            "top",
                            10750,
                            500,
                            "linear",
                            "${axle}",
                            '343px',
                            '267px'
                        ],
                        [
                            "eid350",
                            "top",
                            11250,
                            500,
                            "linear",
                            "${axle}",
                            '267px',
                            '343px'
                        ],
                        [
                            "eid376",
                            "scaleX",
                            12000,
                            500,
                            "linear",
                            "${beams}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid377",
                            "scaleX",
                            12500,
                            500,
                            "linear",
                            "${beams}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid476",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${beams}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid477",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${beams}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid489",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${beams}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid490",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${beams}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid452",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${beams}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid462",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${beams}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid729",
                            "opacity",
                            21000,
                            500,
                            "linear",
                            "${titrewgCopy2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid730",
                            "opacity",
                            21500,
                            500,
                            "linear",
                            "${titrewgCopy2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid731",
                            "opacity",
                            23000,
                            500,
                            "linear",
                            "${titrewgCopy}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid732",
                            "opacity",
                            23500,
                            500,
                            "linear",
                            "${titrewgCopy}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid649",
                            "top",
                            16500,
                            500,
                            "linear",
                            "${capUltra2}",
                            '553px',
                            '500px'
                        ],
                        [
                            "eid650",
                            "top",
                            17020,
                            480,
                            "linear",
                            "${capUltra2}",
                            '500px',
                            '553px'
                        ],
                        [
                            "eid601",
                            "top",
                            0,
                            0,
                            "linear",
                            "${titrewg}",
                            '289px',
                            '289px'
                        ],
                        [
                            "eid686",
                            "opacity",
                            16500,
                            500,
                            "linear",
                            "${capCoul}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid687",
                            "opacity",
                            17000,
                            500,
                            "linear",
                            "${capCoul}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid688",
                            "opacity",
                            18000,
                            500,
                            "linear",
                            "${capCoul}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid689",
                            "opacity",
                            18500,
                            500,
                            "linear",
                            "${capCoul}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid666",
                            "opacity",
                            19500,
                            500,
                            "linear",
                            "${capCoul}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid667",
                            "opacity",
                            20000,
                            500,
                            "linear",
                            "${capCoul}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid539",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${titrewg}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid548",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${titrewg}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid559",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${titrewg}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid565",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${titrewg}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid574",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${titrewg}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid580",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${titrewg}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid602",
                            "left",
                            0,
                            0,
                            "linear",
                            "${titrewb}",
                            '255px',
                            '255px'
                        ],
                        [
                            "eid334",
                            "scaleX",
                            9500,
                            500,
                            "linear",
                            "${wheels}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid335",
                            "scaleX",
                            10000,
                            500,
                            "linear",
                            "${wheels}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid600",
                            "left",
                            0,
                            0,
                            "linear",
                            "${titrewg}",
                            '-142px',
                            '-142px'
                        ],
                        [
                            "eid676",
                            "opacity",
                            15000,
                            500,
                            "linear",
                            "${captTac}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid677",
                            "opacity",
                            15500,
                            500,
                            "linear",
                            "${captTac}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid684",
                            "opacity",
                            16500,
                            500,
                            "linear",
                            "${captTac}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid685",
                            "opacity",
                            17000,
                            500,
                            "linear",
                            "${captTac}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid668",
                            "opacity",
                            19500,
                            500,
                            "linear",
                            "${captTac}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid669",
                            "opacity",
                            20000,
                            500,
                            "linear",
                            "${captTac}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid487",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${wheels}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid488",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${wheels}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid417",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${wheels}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid427",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${wheels}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid448",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${wheels}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid458",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${wheels}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid695",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid696",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid697",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid698",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid699",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid700",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid701",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid702",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${CASE_DEFINITION_YELLOWCopy2}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid474",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${gears}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid475",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${gears}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid503",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${gears}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid494",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${gears}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid420",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${gears}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid430",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${gears}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid609",
                            "left",
                            0,
                            0,
                            "linear",
                            "${gears}",
                            '-79px',
                            '-79px'
                        ],
                        [
                            "eid560",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${titrew}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid566",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${titrew}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid573",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${titrew}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid579",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${titrew}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid588",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${titrew}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid594",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${titrew}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid715",
                            "scaleX",
                            23000,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid716",
                            "scaleX",
                            23500,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid606",
                            "left",
                            0,
                            0,
                            "linear",
                            "${titrew}",
                            '898px',
                            '898px'
                        ],
                        [
                            "eid378",
                            "scaleY",
                            12000,
                            500,
                            "linear",
                            "${beams}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid379",
                            "scaleY",
                            12500,
                            500,
                            "linear",
                            "${beams}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid636",
                            "scaleX",
                            15000,
                            500,
                            "linear",
                            "${capCoul}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid637",
                            "scaleX",
                            15500,
                            500,
                            "linear",
                            "${capCoul}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid743",
                            "scaleX",
                            24500,
                            500,
                            "linear",
                            "${brique}",
                            '1.14999',
                            '1.34'
                        ],
                        [
                            "eid744",
                            "scaleX",
                            25000,
                            500,
                            "linear",
                            "${brique}",
                            '1.34',
                            '1.14999'
                        ],
                        [
                            "eid717",
                            "opacity",
                            21000,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid718",
                            "opacity",
                            21500,
                            500,
                            "linear",
                            "${moteur_moyen}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid739",
                            "top",
                            24500,
                            500,
                            "linear",
                            "${brique}",
                            '462px',
                            '471px'
                        ],
                        [
                            "eid740",
                            "top",
                            25000,
                            500,
                            "linear",
                            "${brique}",
                            '471px',
                            '462px'
                        ],
                        [
                            "eid507",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Imgs}",
                            '-48px',
                            '-48px'
                        ],
                        [
                            "eid705",
                            "scaleX",
                            21000,
                            500,
                            "linear",
                            "${moteur_large}",
                            '1',
                            '1.5'
                        ],
                        [
                            "eid706",
                            "scaleX",
                            21500,
                            500,
                            "linear",
                            "${moteur_large}",
                            '1.5',
                            '1'
                        ],
                        [
                            "eid513",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Imgs}",
                            '-25px',
                            '-25px'
                        ],
                        [
                            "eid611",
                            "left",
                            0,
                            0,
                            "linear",
                            "${wheels}",
                            '883px',
                            '883px'
                        ],
                        [
                            "eid468",
                            "opacity",
                            9500,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '1',
                            '0.3'
                        ],
                        [
                            "eid469",
                            "opacity",
                            10000,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid495",
                            "opacity",
                            10750,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid496",
                            "opacity",
                            11250,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '0.47',
                            '1'
                        ],
                        [
                            "eid418",
                            "opacity",
                            12000,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid428",
                            "opacity",
                            12500,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '0.470000',
                            '1'
                        ],
                        [
                            "eid449",
                            "opacity",
                            13250,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '1',
                            '0.47'
                        ],
                        [
                            "eid459",
                            "opacity",
                            13750,
                            500,
                            "linear",
                            "${Text5Copy}",
                            '0.470000',
                            '1'
                        ]
                    ]
                }
            },
            "TITRE": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['0px', '0px', '574px', '57px', 'auto', 'auto'],
                            font: ['Digital-7', [5, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            align: 'center',
                            id: 'Text6Copy2',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 60px; font-family: \'Digital tech\'; font-weight: 500;\">Materiel a utiliser ?</span></p>',
                            clip: 'rect(0px 571.836669921875px 57px 0px)',
                            type: 'text'
                        },
                        {
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['0px', '0px', '573px', '57px', 'auto', 'auto'],
                            font: ['Digital-7', [100, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            align: 'center',
                            id: 'Text6',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 60px; font-weight: 500; font-family: \'Digital tech\';\">Materiel A utiliser ?</span></p>',
                            clip: 'rect(0px 550px 57px 0px)',
                            type: 'text'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            isStage: 'true',
                            rect: [undefined, undefined, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 4250,
                    autoPlay: true,
                    data: [
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,57,0],
                            [0,60,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,57,0],
                            [0,100,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,100,57,0],
                            [0,140,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,57,0],
                            [0,160,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,57,0],
                            [0,190,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,57,0],
                            [0,220,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,57,0],
                            [0,260,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,57,0],
                            [0,270,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,57,0],
                            [0,290,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,57,0],
                            [0,320,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,57,0],
                            [0,350,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,57,0],
                            [0,380,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,57,0],
                            [0,405,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,57,0],
                            [0,475,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,57,0],
                            [0,505,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,505,57,0],
                            [0,550,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0190_Kits_edgeActions.js");
})("EDGE-6689876");
