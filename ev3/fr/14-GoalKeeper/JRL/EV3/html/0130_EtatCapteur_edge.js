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
                            rect: ['80px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'FONC',
                            type: 'video',
                            tag: 'video',
                            rect: ['378px', '350px', '782px', '447px', 'auto', 'auto'],
                            source: [vid+"FONC%C3%A9.webm"],
                            preload: 'auto'
                        },
                        {
                            id: 'Text3',
                            type: 'group',
                            rect: ['-310px', '114px', '1171', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titre2',
                                type: 'text',
                                rect: ['885px', '172px', '700px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\">Capteur tactile <span style=\"color: rgb(255, 0, 0);\">relaché</span></p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'RELACH22',
                                type: 'video',
                                tag: 'video',
                                rect: ['688px', '236px', '782px', '447px', 'auto', 'auto'],
                                source: [vid+"RELACH2.webm"],
                                preload: 'auto'
                            }]
                        },
                        {
                            id: 'Text7',
                            type: 'group',
                            rect: ['280px', '301px', '1119', '375px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'titre2Copy',
                                type: 'text',
                                rect: ['295px', '-15px', '734px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\">Capteur tactile <span style=\"color: rgb(255, 0, 0);\">heurté</span>​</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: '_0001',
                                type: 'video',
                                tag: 'video',
                                rect: ['98px', '49px', '782px', '447px', 'auto', 'auto'],
                                source: [vid+"0001.webm"],
                                preload: 'auto'
                            }]
                        },
                        {
                            id: 'Text2',
                            type: 'group',
                            rect: ['184px', '415px', '857px', '349px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['391px', '-129px', '627px', '95px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left;\">Capteur tactile <span style=\"color: rgb(255, 0, 0);\">enfoncé</span></p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: '_3Copy',
                            type: 'image',
                            rect: ['1029px', '794px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: '_3',
                            type: 'image',
                            rect: ['1029px', '794px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: 'b3',
                            type: 'image',
                            rect: ['1029px', '794px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b32.png",'0px','0px']
                        },
                        {
                            id: '_2Copy',
                            type: 'image',
                            rect: ['754px', '794px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: '_2',
                            type: 'image',
                            rect: ['754px', '794px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"22.png",'0px','0px']
                        },
                        {
                            id: 'b2',
                            type: 'image',
                            rect: ['754px', '794px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b22.png",'0px','0px']
                        },
                        {
                            id: '_1',
                            type: 'image',
                            rect: ['450px', '791px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"12.png",'0px','0px']
                        },
                        {
                            id: 'b1',
                            type: 'image',
                            rect: ['450px', '791px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"b12.png",'0px','0px']
                        },
                        {
                            id: 'Text',
                            type: 'group',
                            rect: ['354px', '117px', '812px', '169', 'auto', 'auto'],
                            c: [
                            {
                                id: 'Rectangle2',
                                type: 'rect',
                                rect: ['386px', '659px', '130px', '120px', 'auto', 'auto'],
                                fill: ["rgba(192,192,192,0.00)"],
                                stroke: [0,"rgba(0,0,0,0.00)","none"]
                            },
                            {
                                id: 'Rectangle3',
                                type: 'rect',
                                rect: ['663px', '659px', '130px', '120px', 'auto', 'auto'],
                                fill: ["rgba(192,192,192,0.00)"],
                                stroke: [0,"rgba(0,0,0,0.00)","none"]
                            },
                            {
                                id: 'ROBOT08',
                                type: 'image',
                                rect: ['-51px', '3px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['105px', '3px', '756px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'TextIN',
                                type: 'text',
                                rect: ['154px', '37px', '689px', '57px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">Le capteur tactile peut avoir 3 états tel qu'il est expliqué dans les videos ci-dessous&nbsp;</p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Rect_liaison_gris',
                            type: 'rect',
                            rect: ['555px', '839px', '200px', '5px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_vert',
                            type: 'rect',
                            rect: ['554px', '839px', '200px', '0.5%', 'auto', 'auto'],
                            fill: ["rgba(255,170,74,1.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_grisCopy2',
                            type: 'rect',
                            rect: ['861px', '839px', '175px', '5px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,1)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Rect_liaison_vertCopy2',
                            type: 'rect',
                            rect: ['863px', '839px', '170px', '0.5%', 'auto', 'auto'],
                            fill: ["rgba(255,170,74,1.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['795px', '19px', '390', '57', 'auto', 'auto']
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
                        [
                            "eid191",
                            "width",
                            1500,
                            500,
                            "linear",
                            "${Rect_liaison_vertCopy2}",
                            '0px',
                            '170px'
                        ],
                        [
                            "eid182",
                            "width",
                            750,
                            500,
                            "linear",
                            "${Rect_liaison_vert}",
                            '0px',
                            '200px'
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
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-188px', '8px', '859px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 54px; font-weight: 500;\"><span style=\"font-size: 62px;\">etat Capteur</span></p>',
                            font: ['Digital-7', [117, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 900px 57px -196.29614257813px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-188px', '8px', '766px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 54px; font-weight: 500;\"><span style=\"font-size: 62px;\">etat Capteur</span></p>',
                            font: ['Digital-7', [117, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 679.89697265625px 57px -155.555419921875px)',
                            id: 'Text6'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5000,
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
                            [0,515,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,515,57,0],
                            [0,530,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid173",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,-155.555419921875],
                            [0,555,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid175",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,555,57,-155.555419921875],
                            [0,595,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid177",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,-155.555419921875],
                            [0,679.89697265625,57,-155.555419921875],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0130_EtatCapteur_edgeActions.js");
})("EDGE-6689876");
