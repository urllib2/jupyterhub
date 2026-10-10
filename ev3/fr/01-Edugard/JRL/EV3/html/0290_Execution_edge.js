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
                            id: 'TITRE',
                            symbolName: 'TITRE',
                            type: 'rect',
                            rect: ['670px', '22px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'Text3',
                            type: 'group',
                            rect: ['260px', '239px', '1171', '361', 'auto', 'auto'],
                            c: [
                            {
                                id: 'bloc',
                                type: 'image',
                                rect: ['140px', '21px', '848px', '636px', 'auto', 'auto'],
                                clip: 'rect(0px 778.0908203125px 636px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"a1_etapes2.jpg",'0px','0px'],
                                transform: [[],[],[],['0.61733','0.61636']]
                            },
                            {
                                id: 'titre2',
                                type: 'text',
                                rect: ['182px', '49px', '700px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Connecter la brique intélligente avec l'ordinateur.</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'titre2Copy2',
                                type: 'text',
                                rect: ['842px', '176px', '377px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; text-align: left; line-height: 47px;\"><span style=\"color: rgb(0, 0, 0);\">V</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(0, 0, 0);\">ous pouvez utliser :</span></p><p style=\"margin: 0px; text-align: left; line-height: 47px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(0, 0, 0);\">​- Le câble USB</span></p><p style=\"margin: 0px; text-align: left; line-height: 47px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(0, 0, 0);\">​- Le bluetooth</span></p><p style=\"margin: 0px; text-align: left; line-height: 47px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(0, 0, 0);\"></span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; color: rgb(0, 0, 0);\">​- Le WIFI</span></p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [37, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text7',
                            type: 'group',
                            rect: ['359px', '248px', '1119', '375px', 'auto', 'auto'],
                            clip: 'rect(22.982378005981445px 1263.44384765625px 547.3041381835938px -177.6717529296875px)',
                            c: [
                            {
                                id: 'hommes',
                                type: 'image',
                                rect: ['49px', '110px', '752px', '333px', 'auto', 'auto'],
                                clip: 'rect(70.6285629272461px 752px 220.84732055664063px 6.90860652923584px)',
                                fill: ["rgba(0,0,0,0)",im+"1.png",'0px','0px'],
                                transform: [[],[],[],['1.79916','1.89156']]
                            },
                            {
                                id: 'hommesCopy',
                                type: 'image',
                                rect: ['128px', '35px', '752px', '333px', 'auto', 'auto'],
                                clip: 'rect(266.0006408691406px 713.7106323242188px 337.5082702636719px 6.90860652923584px)',
                                fill: ["rgba(0,0,0,0)",im+"1.png",'0px','0px'],
                                transform: [[],[],[],['1.79916','1.89156']]
                            },
                            {
                                id: 'titre2Copy',
                                type: 'text',
                                rect: ['69px', '46px', '734px', '89px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Transfert et exécution du programme</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1.00)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Text2',
                            type: 'group',
                            rect: ['184px', '415px', '857px', '349px', 'auto', 'auto'],
                            c: [
                            {
                                id: 'image_idee',
                                type: 'image',
                                rect: ['335px', '-70px', '599px', '449px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"a1_etapes1.jpeg",'0px','0px']
                            },
                            {
                                id: 'Text4',
                                type: 'text',
                                rect: ['291px', '-128px', '627px', '95px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​Allumer la brique intélligente</p>",
                                align: "center",
                                font: ['Arial, Helvetica, sans-serif', [40, "px"], "rgba(3,55,154,1)", "700", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: '_1',
                            type: 'image',
                            rect: ['453px', '791px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"12.png",'0px','0px']
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
                            id: '_3',
                            type: 'image',
                            rect: ['1033px', '794px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: 'b1',
                            type: 'image',
                            rect: ['453px', '791px', '105px', '105px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"b12.png",'0px','0px']
                        },
                        {
                            id: 'b2',
                            type: 'image',
                            rect: ['754px', '793px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b22.png",'0px','0px']
                        },
                        {
                            id: '_3Copy',
                            type: 'image',
                            rect: ['1032px', '793px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"32.png",'0px','0px']
                        },
                        {
                            id: 'b3',
                            type: 'image',
                            rect: ['1032px', '794px', '105px', '105px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"b32.png",'0px','0px']
                        },
                        {
                            id: 'cach2',
                            type: 'rect',
                            rect: ['749px', '785px', '129px', '114px', 'auto', 'auto'],
                            fill: ["rgba(195,195,195,0.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'cach3',
                            type: 'rect',
                            rect: ['1021px', '789px', '129px', '114px', 'auto', 'auto'],
                            fill: ["rgba(192,192,192,0.00)"],
                            stroke: [0,"rgba(0,0,0,1)","none"]
                        },
                        {
                            id: 'Text',
                            type: 'group',
                            rect: ['327px', '124px', '812px', '169', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ROBOT08',
                                type: 'image',
                                rect: ['-35px', '-7px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotR.png",'0px','0px']
                            },
                            {
                                id: 'CASE_DEFINITION_GREEN',
                                type: 'image',
                                rect: ['105px', '2px', '892px', '128px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_GREEN1.png",'0px','0px']
                            },
                            {
                                id: 'TextIN',
                                type: 'text',
                                rect: ['149px', '19px', '832px', '141px', 'auto', 'auto'],
                                opacity: '1',
                                text: "<p style=\"margin: 0px;\"><span style=\"font-size: 36px;\">Le programme doit être transféré de l'ordinateur vers la brique intelligente afin de l'exécuter.</span><span style=\"font-size: 35px;\">&nbsp;</span></p>",
                                align: "left",
                                font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [34, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'pointer',
                            type: 'image',
                            rect: ['529px', '856px', '38px', '52px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"pointer.png",'0px','0px']
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
                            id: 'retro-robot-vector-illustration',
                            type: 'image',
                            rect: ['260px', '271px', '1095px', '517px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"retro-robot-vector-illustration.png",'0px','0px']
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
                            "eid182",
                            "width",
                            750,
                            500,
                            "linear",
                            "${Rect_liaison_vert}",
                            '0px',
                            '200px'
                        ],
                        [
                            "eid332",
                            "background-color",
                            5500,
                            0,
                            "linear",
                            "${cach2}",
                            'rgba(195,195,195,0.00)',
                            'rgba(195,195,195,0.00)'
                        ],
                        [
                            "eid191",
                            "width",
                            1500,
                            500,
                            "linear",
                            "${Rect_liaison_vertCopy2}",
                            '0px',
                            '170px'
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
                            font: ['Digital-7', [117, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['-188px', '8px', '859px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 900px 57px -196.29614257813px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 54px; font-family: \'Digital tech\'; font-weight: 500;\">execution d</span><span style=\"color: rgb(0, 61, 65); font-size: 54px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500;\">\'</span><span style=\"color: rgb(0, 61, 65); font-size: 54px; font-family: \'Digital tech\'; font-weight: 500;\">un programme</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [117, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['-188px', '8px', '766px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 679.89697265625px 57px -155.555419921875px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 54px; font-family: \'Digital tech\'; font-weight: 500;\">execution d</span><span style=\"font-size: 54px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500;\">\'</span><span style=\"font-size: 54px; font-family: \'Digital tech\'; font-weight: 500;\">un programme</span></p>'
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0290_Execution_edgeActions.js");
})("EDGE-6689876");
