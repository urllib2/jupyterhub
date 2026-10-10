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
                scaleToFit: "both",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['86px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'presentBrick',
                            type: 'video',
                            tag: 'video',
                            rect: ['447px', '110px', '1364px', '1066px', 'auto', 'auto'],
                            clip: 'rect(0px 1077.333984375px 854.888671875px 413.3330078125px)',
                            source: [vid+"onglets.webm"],
                            preload: 'auto',
                            transform: [[],[],[],['0.96653','0.96563']]
                        },
                        {
                            id: 'Text',
                            type: 'group',
                            rect: ['63px', '198px', '803', '169', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ROBOT08',
                                type: 'image',
                                rect: ['51px', '-36px', '150px', '150px', 'auto', 'auto'],
                                clip: 'rect(0px 150px 171.5px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'-0.41672620960509px','7.9168545863481px'],
                                transform: [[],[],[],['1.06667','1.06667']]
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['176px', '-55px', '648px', '209px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'TextIN',
                                type: 'text',
                                rect: ['228px', '-41px', '581px', '250px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\">L'interface de la brique propose quatre écrans de base. &nbsp; </p><p style=\"margin: 0px;\">La navigation est possible grâce aux boutons droit et gauche de la brique.</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text1',
                            type: 'group',
                            rect: ['217px', '285px', '1172', '477', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titreCopy4',
                                type: 'text',
                                rect: ['122px', '106px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Programmes récents</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text4Copy',
                                type: 'text',
                                rect: ['154px', '169px', '509px', '391px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Ici sont affichés les programmes récemment exécutés. &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; <br>Le premier programme de la liste est sélectionné par</p><p style=\"margin: 0px;\">défaut. Il s'agit du dernier programme exécuté.</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text2',
                            type: 'group',
                            rect: ['155px', '278px', '1172', '477', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titre',
                                type: 'text',
                                rect: ['203px', '116px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Explorateur de fichiers</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['214px', '177px', '453px', '306px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Cet écran permet d'accéder à tous les fichiers de la brique EV3 et les gérer (déplacer ou supprimer).</p><p style=\"margin: 0px;\">Les fichiers sont organisés dans des dossiers de projet qui contiennent les fichiers du programme.</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text3',
                            type: 'group',
                            rect: ['-273px', '357px', '1171', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'text',
                                type: 'text',
                                rect: ['643px', '94px', '477px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 34px;\">Quatre applis prêtes à l'emploi sont préinstallées sur la brique EV3 qui sont :​</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">​</span><span style=\"font-size: 34px; font-weight: 700;\">- Port View</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 700;\">​-​ Motor Control</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 700;\">​- IR Control</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 700;\">​- Programmation sur la brique</span></p><p style=\"margin: 0px;\"></p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titreCopy',
                                type: 'text',
                                rect: ['595px', '38px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Applis de la brique\n​</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text7',
                            type: 'group',
                            rect: ['195px', '229px', '1119', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'textee',
                                type: 'text',
                                rect: ['177px', '220px', '459px', '165px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 34px;\">Cet écran permet d'afficher et de modifier divers paramètres</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px;\">généraux sur la brique EV3 tels que :</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 700;\">- Volume</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 700;\">​- Bluetooth</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 700;\">​- Wifi</span></p><p style=\"margin: 0px;\"><span style=\"font-size: 34px; font-weight: 700;\">​- Nom de la brique</span></p><p style=\"margin: 0px;\">​</p>",
                                align: "left",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [32, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titreCopy2',
                                type: 'text',
                                rect: ['83px', '163px', '452px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Branchement</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: '_1',
                            type: 'image',
                            rect: ['226px', '427px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"12.png",'0px','0px']
                        },
                        {
                            id: 'b1',
                            type: 'image',
                            rect: ['226px', '427px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"b12.png",'0px','0px']
                        },
                        {
                            id: '_2',
                            type: 'image',
                            rect: ['226px', '535px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: 'b2',
                            type: 'image',
                            rect: ['226px', '535px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"b22.png",'0px','0px']
                        },
                        {
                            id: '_2Copy',
                            type: 'image',
                            rect: ['226px', '535px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: '_3',
                            type: 'image',
                            rect: ['226px', '640px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: 'b3',
                            type: 'image',
                            rect: ['226px', '640px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"b32.png",'0px','0px']
                        },
                        {
                            id: '_3Copy',
                            type: 'image',
                            rect: ['226px', '640px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: '_4',
                            type: 'image',
                            rect: ['226px', '746px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"42.png",'0px','0px']
                        },
                        {
                            id: 'b4',
                            type: 'image',
                            rect: ['226px', '746px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'default',
                            fill: ["rgba(0,0,0,0)",im+"b42.png",'0px','0px']
                        },
                        {
                            id: '_4Copy',
                            type: 'image',
                            rect: ['226px', '746px', '105px', '80px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"42.png",'0px','0px']
                        },
                        {
                            id: 'pointer',
                            type: 'image',
                            rect: ['314px', '477px', '34px', '42px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'cache_cadre',
                            type: 'image',
                            rect: ['86px', '878px', '1449px', '202px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
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
                    duration: 5500,
                    autoPlay: false,
                    data: [

                    ]
                }
            },
            "TITRE_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['-68px', '-3px', '681px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 714.164306640625px 57px -45.71435546875px)',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\"><span style=\"font-size: 60px;\">Interface de la brique</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['MS Serif, New York, serif', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['-39px', '-3px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 680.7080078125px 57px -78.28271484375px)',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\"><span style=\"font-size: 60px;\">Interface de la brique</span></p>'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5250,
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
                            [0,530,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid199",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,0],
                            [0,540,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid201",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,540,57,0],
                            [0,565,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid203",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,565,57,0],
                            [0,595,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid205",
                            "clip",
                            5000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,0],
                            [0,680.7080078125,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0170_Navigation_edgeActions.js");
})("EDGE-6689876");
