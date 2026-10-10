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
                scaleToFit: "none",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['85px', '0px', '1440px', '1080px', 'auto', 'auto'],
                            clip: 'rect(0px 1440px 1080px 0px)',
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px'],
                            transform: [[],[],[],['1.00694']]
                        },
                        {
                            id: 'Intro',
                            type: 'group',
                            rect: ['263px', '101px', '1128px', '177', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['121px', '22px', '900px', '187px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET1.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['166px', '49px', '613px', '157px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">We propose to program the robot so that it emits the sound \"Hello\" and then display the following image.​</p>",
                                align: "left",
                                font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'ROBOT07',
                                type: 'image',
                                rect: ['-25px', '49px', '160px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotE.png",'0px','0px']
                            },
                            {
                                id: 'a3_image',
                                type: 'image',
                                rect: ['779px', '48px', '192px', '143px', 'auto', 'auto'],
                                clip: 'rect(0px 192px 141.161865234375px 0px)',
                                fill: ["rgba(0,0,0,0)",im+"a3_image.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'cache_cadre',
                            type: 'image',
                            rect: ['80px', '878px', '1449px', '202px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                        },
                        {
                            id: 'question',
                            type: 'group',
                            rect: ['441px', '354px', '769', '120', 'auto', 'auto'],
                            cursor: 'default',
                            c: [
                            {
                                id: 'camarade',
                                type: 'group',
                                rect: ['-65px', '316px', '1050', '140px', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'Rectangle',
                                    type: 'rect',
                                    rect: ['-16px', '11px', '925px', '118px', 'auto', 'auto'],
                                    fill: ["rgba(93,216,223,0.1294)"],
                                    stroke: [1,"rgba(0,0,0,1)","solid"]
                                },
                                {
                                    id: 'Text',
                                    type: 'text',
                                    rect: ['0px', '48px', '824px', '93px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-size: 36px;\">Did the program perform the mission correctly?</span></p>",
                                    align: "left",
                                    font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                },
                                {
                                    id: 'rep1_yes',
                                    type: 'image',
                                    rect: ['803px', '27px', '94px', '85px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_VERT2.png",'0px','0px']
                                },
                                {
                                    id: 'rep1_no',
                                    type: 'image',
                                    rect: ['803px', '27px', '94px', '85px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_ROUGE2.png",'0px','0px']
                                },
                                {
                                    id: 'rep_1',
                                    type: 'image',
                                    rect: ['803px', '27px', '94px', '85px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"CARR_GRIS.png",'0px','0px']
                                },
                                {
                                    id: 'Yes',
                                    type: 'image',
                                    rect: ['1000px', '45px', '50px', '50px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"Yes2.png",'0px','0px']
                                },
                                {
                                    id: 'No_1',
                                    type: 'image',
                                    rect: ['934px', '45px', '50px', '50px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"No2.png",'0px','0px']
                                }]
                            },
                            {
                                id: 'Hint_green',
                                type: 'image',
                                rect: ['-81px', '-14px', '938px', '329px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"Hint_green2.png",'0px','0px']
                            },
                            {
                                id: 'Text6',
                                type: 'text',
                                rect: ['-12px', '11px', '824px', '75px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 36px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"></span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 36px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"></span>To do this, one of your comrade proposed the following solution:<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 36px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 36px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 36px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 36px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 36px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span>Test this program then answer this question<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"></span></p><p style=\"margin: 0px; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-indent: 0px; line-height: 35px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"></span></p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'A3_camarade',
                                type: 'image',
                                rect: ['96px', '77px', '617px', '157px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"A3_camarade.png",'0px','0px']
                            },
                            {
                                id: 'valider',
                                type: 'group',
                                rect: ['-59px', '453px', '1040', '82', 'auto', 'auto'],
                                clip: 'rect(0px 1040px 82px 900px)',
                                c: [
                                {
                                    id: 'ORANGECopy',
                                    type: 'image',
                                    rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE3.png",'0px','0px']
                                },
                                {
                                    id: 'GREYCopy',
                                    type: 'image',
                                    rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                    cursor: 'pointer',
                                    fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY3.png",'0px','0px']
                                },
                                {
                                    id: 'Text3Copy',
                                    type: 'text',
                                    rect: ['910px', '13px', '124px', '50px', 'auto', 'auto'],
                                    cursor: 'pointer',
                                    text: "<p style=\"margin: 0px;\">​ &nbsp;Validate</p>",
                                    align: "left",
                                    font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            }]
                        },
                        {
                            id: 'demo',
                            type: 'group',
                            rect: ['-141px', '340px', '1051px', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1054.28564453125px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE2.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY2.png",'0px','0px']
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['921px', '19px', '113px', '32px', 'auto', 'auto'],
                                clip: 'rect(5.479248046875px 116.58056640625px 45.478271484375px -2.97216796875px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-size: 26px;\">Question</span></p>",
                                align: "left",
                                font: ['MagistralC', [0, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'Explication',
                            type: 'text',
                            rect: ['389px', '689px', '894px', '29px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; line-height: 38px;\">​This program does not perform the mission correctly since it displays the image on the screen of the brick for a very short time. .........</p><p style=\"margin: 0px; line-height: 38px;\"><span style=\"font-size: 35px;\"></span></p><p style=\"margin: 0px; line-height: 38px;\"><span style=\"font-size: 35px;\">​</span></p>",
                            align: "left",
                            font: ['Lucida Sans Unicode, Lucida Grande, sans-serif', [40, "px"], "rgba(139,64,152,1.00)", "700", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['692px', '29px', '390', '57', 'auto', 'auto']
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
                    duration: 5000,
                    autoPlay: false,
                    data: [
                        [
                            "eid480",
                            "background-color",
                            0,
                            0,
                            "linear",
                            "${Rectangle}",
                            'rgba(93,216,223,0.13)',
                            'rgba(93,216,223,0.13)'
                        ],
                        [
                            "eid483",
                            "left",
                            0,
                            0,
                            "linear",
                            "${Rectangle}",
                            '-16px',
                            '-16px'
                        ],
                        [
                            "eid487",
                            "height",
                            0,
                            0,
                            "linear",
                            "${Rectangle}",
                            '118px',
                            '118px'
                        ],
                        [
                            "eid488",
                            "top",
                            0,
                            0,
                            "linear",
                            "${Rectangle}",
                            '11px',
                            '11px'
                        ]
                    ]
                }
            },
            "video": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [

                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '1920px', '1080px']
                        }
                    }
                },
                timeline: {
                    duration: 1500,
                    autoPlay: true,
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
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-4px', '0px', '859px', '57px', 'auto', 'auto'],
                            font: ['Digital-7', [117, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            align: 'center',
                            id: 'Text6Copy2',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Activity</span></p>',
                            clip: 'rect(0px 900px 57px -196.29614257813px)',
                            type: 'text'
                        },
                        {
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-4px', '-1px', '766px', '57px', 'auto', 'auto'],
                            font: ['Digital-7', [117, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            align: 'center',
                            id: 'Text6',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Activity</span></p>',
                            clip: 'rect(0px 650px 57px -155.55541992188px)',
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
                            [0,650,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid476",
                            "top",
                            5000,
                            0,
                            "linear",
                            "${Text6}",
                            '-1px',
                            '-1px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0170_AffichageCamarade_edgeActions.js");
})("EDGE-6689876");
