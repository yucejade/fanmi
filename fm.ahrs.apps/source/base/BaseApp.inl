#include <Urho3D/Core/Timer.h>
#include <Urho3D/Engine/Application.h>
#include <Urho3D/Engine/Console.h>
#include <Urho3D/Engine/DebugHud.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Engine/EngineDefs.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/GraphicsAPI/Texture2D.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Input/InputEvents.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Resource/XMLFile.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/SceneEvents.h>
#include <Urho3D/UI/Cursor.h>
#include <Urho3D/UI/Sprite.h>
#include <Urho3D/UI/UI.h>

//

BaseApp::BaseApp( Context* context ) : Application( context ), yaw_( 0.0f ), pitch_( 0.0f ), touchEnabled_( false ), useMouseMode_( MM_ABSOLUTE ), screenJoystickIndex_( M_MAX_UNSIGNED ), screenJoystickSettingsIndex_( M_MAX_UNSIGNED ), paused_( false ) {}
//
void BaseApp::Setup()
{
    // Modify engine startup parameters
    engineParameters_[ EP_WINDOW_TITLE ]   = GetTypeName();
    engineParameters_[ EP_LOG_NAME ]       = GetSubsystem< FileSystem >()->GetProgramDir() + GetTypeName() + ".log";
    engineParameters_[ EP_FULL_SCREEN ]    = false;
    engineParameters_[ EP_HEADLESS ]       = false;
    engineParameters_[ EP_SOUND ]          = false;
    engineParameters_[ EP_RESOURCE_PATHS ] = "CoreData;Data;UserData;";
    // 设置引擎参数
    // engineParameters_[ "FullScreen" ]      = true;  // 设置为全屏模式
    engineParameters_[ "WindowResizable" ] = true;  // 允许窗口调整大小

    // Construct a search path to find the resource prefix with two entries:
    // The first entry is an empty path which will be substituted with program/bin directory -- this entry is for binary when it is still in build tree
    // The second and third entries are possible relative paths from the installed program/bin directory to the asset directory -- these entries are for binary when it is in the Urho3D SDK installation location
    if ( ! engineParameters_.Contains( EP_RESOURCE_PREFIX_PATHS ) )
    {
        engineParameters_[ EP_RESOURCE_PREFIX_PATHS ] = ";../share/Resources;../share/Urho3D/Resources";
    }
}
//
void BaseApp::Start()
{
    if ( GetPlatform() == "Android" || GetPlatform() == "iOS" )
    {  // On mobile platform, enable touch by adding a screen joystick
        InitTouchInput();
    }
    else if ( GetSubsystem< Input >()->GetNumJoysticks() == 0 )
    {  // On desktop platform, do not detect touch when we already got a joystick
        SubscribeToEvent( E_TOUCHBEGIN, URHO3D_HANDLER( BaseApp, HandleTouchBegin ) );
    }
    // Create logo
    CreateLogo();

    // Set custom window Title & Icon
    SetWindowTitleAndIcon();

    // Create console and debug HUD
    CreateConsoleAndDebugHud();

    // Subscribe key down event
    SubscribeToEvent( E_KEYDOWN, URHO3D_HANDLER( BaseApp, HandleKeyDown ) );
    // Subscribe key up event
    SubscribeToEvent( E_KEYUP, URHO3D_HANDLER( BaseApp, HandleKeyUp ) );
    // Subscribe scene update event
    SubscribeToEvent( E_SCENEUPDATE, URHO3D_HANDLER( BaseApp, HandleSceneUpdate ) );
}

void BaseApp::Stop()
{
    engine_->DumpResources( true );
}

void BaseApp::InitTouchInput()
{
    touchEnabled_ = true;

    ResourceCache* cache       = GetSubsystem< ResourceCache >();
    Input*         input       = GetSubsystem< Input >();
    XMLFile*       layout      = cache->GetResource< XMLFile >( "UI/ScreenJoystick_Samples.xml" );
    const String&  patchString = GetScreenJoystickPatchString();
    if ( ! patchString.Empty() )
    {
        // Patch the screen joystick layout further on demand
        SharedPtr< XMLFile > patchFile( new XMLFile( context_ ) );
        if ( patchFile->FromString( patchString ) )
        {
            layout->Patch( patchFile );
        }
    }
    screenJoystickIndex_ = ( unsigned )input->AddScreenJoystick( layout, cache->GetResource< XMLFile >( "UI/DefaultStyle.xml" ) );
    input->SetScreenJoystickVisible( screenJoystickSettingsIndex_, true );
}

void BaseApp::InitMouseMode( MouseMode mode )
{
    useMouseMode_ = mode;

    Input* input = GetSubsystem< Input >();

    if ( GetPlatform() != "Web" )
    {
        if ( useMouseMode_ == MM_FREE )
        {
            input->SetMouseVisible( true );
        }

        Console* console = GetSubsystem< Console >();
        if ( useMouseMode_ != MM_ABSOLUTE )
        {
            input->SetMouseMode( useMouseMode_ );
            if ( console && console->IsVisible() )
            {
                input->SetMouseMode( MM_ABSOLUTE, true );
            }
        }
    }
    else
    {
        input->SetMouseVisible( true );
        SubscribeToEvent( E_MOUSEBUTTONDOWN, URHO3D_HANDLER( BaseApp, HandleMouseModeRequest ) );
        SubscribeToEvent( E_MOUSEMODECHANGED, URHO3D_HANDLER( BaseApp, HandleMouseModeChange ) );
    }
}

void BaseApp::SetLogoVisible( bool enable )
{
    if ( logoSprite_ )
    {
        logoSprite_->SetVisible( enable );
    }
}

void BaseApp::CreateLogo()
{
    // Get logo texture
    ResourceCache* cache       = GetSubsystem< ResourceCache >();
    Texture2D*     logoTexture = cache->GetResource< Texture2D >( "Textures/logo_outlined.png" );
    if ( ! logoTexture )
    {
        return;
    }
    // Create logo sprite and add to the UI layout
    UI* ui      = GetSubsystem< UI >();
    logoSprite_ = ui->GetRoot()->CreateChild< Sprite >();

    // Set logo sprite texture
    logoSprite_->SetTexture( logoTexture );

    int textureWidth  = logoTexture->GetWidth();
    int textureHeight = logoTexture->GetHeight();

    // Set logo sprite scale
    logoSprite_->SetScale( 128.0f / textureWidth );

    // Set logo sprite size
    logoSprite_->SetSize( textureWidth, textureHeight );

    // Set logo sprite hot spot
    logoSprite_->SetHotSpot( textureWidth, textureHeight );

    // Set logo sprite alignment
    logoSprite_->SetAlignment( HA_RIGHT, VA_BOTTOM );

    // Make logo not fully opaque to show the scene underneath
    logoSprite_->SetOpacity( 0.7f );

    // Set a low priority for the logo so that other UI elements can be drawn on top
    logoSprite_->SetPriority( -100 );
}

void BaseApp::SetWindowTitleAndIcon()
{
    ResourceCache* cache    = GetSubsystem< ResourceCache >();
    Graphics*      graphics = GetSubsystem< Graphics >();
    Image*         icon     = cache->GetResource< Image >( "Textures/logo_outlined.png" );
    graphics->SetWindowIcon( icon );
    graphics->SetWindowTitle( "Ahrs" );
}

void BaseApp::CreateConsoleAndDebugHud()
{
    // Get default style
    ResourceCache* cache   = GetSubsystem< ResourceCache >();
    XMLFile*       xmlFile = cache->GetResource< XMLFile >( "UI/DefaultStyle.xml" );

    // Create console
    Console* console = engine_->CreateConsole();
    console->SetDefaultStyle( xmlFile );
    console->GetBackground()->SetOpacity( 0.8f );

    // Create debug HUD.
    DebugHud* debugHud = engine_->CreateDebugHud();
    debugHud->SetDefaultStyle( xmlFile );
}

void BaseApp::HandleKeyUp( StringHash /*eventType*/, VariantMap& eventData )
{
    using namespace KeyUp;

    int key = eventData[ P_KEY ].GetI32();

    // Close console (if open) or exit when ESC is pressed
    if ( key == KEY_ESCAPE )
    {

        // If the console is open, close it
        // If the console is not open, exit the application
        Console* console = GetSubsystem< Console >();
        if ( console->IsVisible() )
        {
            console->SetVisible( false );
        }
        else
        {
            if ( GetPlatform() == "Web" )
            {
                GetSubsystem< Input >()->SetMouseVisible( true );
                if ( useMouseMode_ != MM_ABSOLUTE )
                {
                    GetSubsystem< Input >()->SetMouseMode( MM_FREE );
                }
            }
            else
            {
                VariantMap& eventData = GetEventDataMap();
                SendEvent( E_EXITREQUESTED, eventData );
                //
                engine_->Exit();
            }
        }
    }
}

void BaseApp::HandleKeyDown( StringHash /*eventType*/, VariantMap& eventData )
{
    using namespace KeyDown;

    int key = eventData[ P_KEY ].GetI32();

    // Toggle console with F1
    if ( key == KEY_F1 )
    {
        GetSubsystem< Console >()->Toggle();
    }
    // Toggle debug HUD with F2
    else if ( key == KEY_F2 )
    {
        GetSubsystem< DebugHud >()->ToggleAll();
    }
    // Common rendering quality controls, only when UI has no focused element
    else if ( ! GetSubsystem< UI >()->GetFocusElement() )
    {
        Renderer* renderer = GetSubsystem< Renderer >();

        // Preferences / Pause
        if ( key == KEY_SELECT && touchEnabled_ )
        {
            paused_ = ! paused_;

            Input* input = GetSubsystem< Input >();
            if ( screenJoystickSettingsIndex_ == M_MAX_UNSIGNED )
            {
                // Lazy initialization
                ResourceCache* cache         = GetSubsystem< ResourceCache >();
                screenJoystickSettingsIndex_ = ( unsigned )input->AddScreenJoystick( cache->GetResource< XMLFile >( "UI/ScreenJoystickSettings_Samples.xml" ), cache->GetResource< XMLFile >( "UI/DefaultStyle.xml" ) );
            }
            else
            {
                input->SetScreenJoystickVisible( screenJoystickSettingsIndex_, paused_ );
            }
        }

        // Texture quality
        else if ( key == '1' )
        {
            auto quality = ( unsigned )renderer->GetTextureQuality();
            ++quality;
            if ( quality > QUALITY_HIGH )
            {
                quality = QUALITY_LOW;
            }
            renderer->SetTextureQuality( ( MaterialQuality )quality );
        }

        // Material quality
        else if ( key == '2' )
        {
            auto quality = ( unsigned )renderer->GetMaterialQuality();
            ++quality;
            if ( quality > QUALITY_HIGH )
            {
                quality = QUALITY_LOW;
            }
            renderer->SetMaterialQuality( ( MaterialQuality )quality );
        }

        // Specular lighting
        else if ( key == '3' )
        {
            renderer->SetSpecularLighting( ! renderer->GetSpecularLighting() );
        }
        // Shadow rendering
        else if ( key == '4' )
        {
            renderer->SetDrawShadows( ! renderer->GetDrawShadows() );
        }
        // Shadow map resolution
        else if ( key == '5' )
        {
            int shadowMapSize = renderer->GetShadowMapSize();
            shadowMapSize *= 2;
            if ( shadowMapSize > 2048 )
            {
                shadowMapSize = 512;
            }
            renderer->SetShadowMapSize( shadowMapSize );
        }

        // Shadow depth and filtering quality
        else if ( key == '6' )
        {
            ShadowQuality quality = renderer->GetShadowQuality();
            quality               = ( ShadowQuality )( quality + 1 );
            if ( quality > SHADOWQUALITY_BLUR_VSM )
            {
                quality = SHADOWQUALITY_SIMPLE_16BIT;
            }
            renderer->SetShadowQuality( quality );
        }

        // Occlusion culling
        else if ( key == '7' )
        {
            bool occlusion = renderer->GetMaxOccluderTriangles() > 0;
            occlusion      = ! occlusion;
            renderer->SetMaxOccluderTriangles( occlusion ? 5000 : 0 );
        }

        // Instancing
        else if ( key == '8' )
        {
            renderer->SetDynamicInstancing( ! renderer->GetDynamicInstancing() );
        }
        // Take screenshot
        else if ( key == '9' )
        {
            Graphics* graphics = GetSubsystem< Graphics >();
            Image     screenshot( context_ );
            graphics->TakeScreenShot( screenshot );
            // Here we save in the Data folder with date and time appended
            screenshot.SavePNG( GetSubsystem< FileSystem >()->GetProgramDir() + "Data/Screenshot_" + Time::GetTimeStamp().Replaced( ':', '_' ).Replaced( '.', '_' ).Replaced( ' ', '_' ) + ".png" );
        }
    }
}

void BaseApp::HandleSceneUpdate( StringHash /*eventType*/, VariantMap& eventData )
{
    // Move the camera by touch, if the camera node is initialized by descendant sample class
    if ( touchEnabled_ && cameraNode_ )
    {
        Input* input = GetSubsystem< Input >();
        for ( i32 i = 0; i < input->GetNumTouches(); ++i )
        {
            TouchState* state = input->GetTouch( i );
            if ( ! state->touchedElement_ )  // Touch on empty space
            {
                if ( state->delta_.x_ || state->delta_.y_ )
                {
                    Camera* camera = cameraNode_->GetComponent< Camera >();
                    if ( ! camera )
                    {
                        return;
                    }
                    Graphics* graphics = GetSubsystem< Graphics >();
                    yaw_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.x_;
                    pitch_ += TOUCH_SENSITIVITY * camera->GetFov() / graphics->GetHeight() * state->delta_.y_;

                    // Construct new orientation for the camera scene node from yaw and pitch; roll is fixed to zero
                    cameraNode_->SetRotation( Quaternion( pitch_, yaw_, 0.0f ) );
                }
                else
                {
                    // Move the cursor to the touch position
                    Cursor* cursor = GetSubsystem< UI >()->GetCursor();
                    if ( cursor && cursor->IsVisible() )
                    {
                        cursor->SetPosition( state->position_ );
                    }
                }
            }
        }
    }
}

void BaseApp::HandleTouchBegin( StringHash /*eventType*/, VariantMap& eventData )
{
    // On some platforms like Windows the presence of touch input can only be detected dynamically
    InitTouchInput();
    UnsubscribeFromEvent( "TouchBegin" );
}

// If the user clicks the canvas, attempt to switch to relative mouse mode on web platform
void BaseApp::HandleMouseModeRequest( StringHash /*eventType*/, VariantMap& eventData )
{
    Console* console = GetSubsystem< Console >();
    if ( console && console->IsVisible() )
    {
        return;
    }
    Input* input = GetSubsystem< Input >();
    if ( useMouseMode_ == MM_ABSOLUTE )
    {
        input->SetMouseVisible( false );
    }
    else if ( useMouseMode_ == MM_FREE )
    {
        input->SetMouseVisible( true );
    }
    input->SetMouseMode( useMouseMode_ );
}

void BaseApp::HandleMouseModeChange( StringHash /*eventType*/, VariantMap& eventData )
{
    Input* input       = GetSubsystem< Input >();
    bool   mouseLocked = eventData[ MouseModeChanged::P_MOUSELOCKED ].GetBool();
    input->SetMouseVisible( ! mouseLocked );
}
