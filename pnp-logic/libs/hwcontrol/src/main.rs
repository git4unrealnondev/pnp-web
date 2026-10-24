use std::io::{self, Stdout};
use std::time::{Duration, Instant};

use hwcontrol::transport::SerialTransport;
use hwcontrol::{
    JogRequest, LightColor, Machine, MessageBuilder, NozzleId, OpenKayoMachine, Position,
};

use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEvent, KeyEventKind, KeyModifiers};
use ratatui::crossterm::execute;
use ratatui::crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, List, ListItem, Paragraph, Wrap};

type TerminalType = Terminal<CrosstermBackend<Stdout>>;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Menu,
    Input,
}

#[derive(Clone, Copy)]
enum InputTarget {
    Port,
    MoveXy,
    MoveZ,
    Jog,
    Rotate,
    Feeder,
    Light,
    Track,
}

struct App {
    machine: Option<Box<dyn Machine>>,
    status: String,
    log: Vec<String>,
    mode: Mode,
    input_buf: String,
    input_prompt: String,
    input_target: Option<InputTarget>,
    last_input_error: Option<String>,
    active_nozzle: NozzleId,
    position: Option<Position>,
}

impl App {
    fn new() -> Self {
        Self {
            machine: None,
            status: "Disconnected. [P] open serial port  [L] loopback  [Q] quit".into(),
            log: Vec::new(),
            mode: Mode::Menu,
            input_buf: String::new(),
            input_prompt: String::new(),
            input_target: None,
            last_input_error: None,
            active_nozzle: NozzleId::N0,
            position: None,
        }
    }

    fn log(&mut self, msg: impl Into<String>) {
        let line = format!(">> {}", msg.into());
        self.log.push(line);
        if self.log.len() > 500 {
            let drop = self.log.len() - 500;
            self.log.drain(0..drop);
        }
    }
}

fn main() -> io::Result<()> {
    enable_raw_mode()?;
    execute!(io::stdout(), EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    let mut app = App::new();
    let res = run_app(&mut terminal, &mut app);

    disable_raw_mode()?;
    execute!(io::stdout(), LeaveAlternateScreen)?;
    res
}

fn run_app(terminal: &mut TerminalType, app: &mut App) -> io::Result<()> {
    let mut last_tick = Instant::now();
    let tick_rate = Duration::from_millis(250);

    loop {
        terminal.draw(|f| ui(f, app))?;

        let timeout = tick_rate.saturating_sub(last_tick.elapsed());
        if event::poll(timeout)?
            && let Event::Key(key) = event::read()?
        {
            if key.kind != KeyEventKind::Press {
                continue;
            }
            match app.mode {
                Mode::Menu => handle_menu_key(app, key)?,
                Mode::Input => handle_input_key(app, key)?,
            }
            if app.status == "__QUIT__" {
                return Ok(());
            }
        }
        if last_tick.elapsed() >= tick_rate {
            last_tick = Instant::now();
        }
    }
}

fn quit(app: &mut App) {
    app.status = "__QUIT__".into();
}

fn handle_menu_key(app: &mut App, key: KeyEvent) -> io::Result<()> {
    if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('c') {
        quit(app);
        return Ok(());
    }
    let n = app.active_nozzle;
    match key.code {
        KeyCode::Char('q') | KeyCode::Esc => quit(app),
        KeyCode::Char('p') => {
            start_input(app, "Serial port (e.g. /dev/ttyUSB0):", InputTarget::Port)
        }
        KeyCode::Char('l') => connect_loopback(app),
        KeyCode::Char('d') => disconnect(app),
        KeyCode::Char('h') => run(app, "home_all", |m| m.home_all()),
        KeyCode::Char('H') => run(app, "home_z", |m| m.home_z()),
        KeyCode::Char('r') => run(app, "reset", |m| m.reset()),
        KeyCode::Char('s') => report_position(app),
        KeyCode::Char('x') => start_input(
            app,
            "move XY: x y speed [NORMAL|T1|T2|T3|THROW]",
            InputTarget::MoveXy,
        ),
        KeyCode::Char('z') => start_input(
            app,
            &format!("move Z: height speed   (nozzle N{:?})", n),
            InputTarget::MoveZ,
        ),
        KeyCode::Char('j') => start_input(app, "jog: x[-1|0|1] y[-1|0|1] speed", InputTarget::Jog),
        KeyCode::Char('R') => start_input(
            app,
            &format!("rotate: angle speed   (nozzle N{:?})", n),
            InputTarget::Rotate,
        ),
        KeyCode::Char('v') => run(app, &format!("open_vac N{:?}", n), move |m| {
            m.open_vacuum(n)
        }),
        KeyCode::Char('V') => run(app, &format!("close_vac N{:?}", n), move |m| {
            m.close_vacuum(n)
        }),
        KeyCode::Char('b') => run(app, &format!("open_blow N{:?}", n), move |m| m.open_blow(n)),
        KeyCode::Char('B') => run(app, &format!("close_blow N{:?}", n), move |m| {
            m.close_blow(n)
        }),
        KeyCode::Char('c') => run(app, &format!("vac->blow N{:?}", n), move |m| {
            m.close_vacuum_open_blow(n)
        }),
        KeyCode::Char('C') => run(app, "close vac+blow all", |m| m.close_vacuum_blow_all()),
        KeyCode::Char('o') => run(app, "open_feeder 0", |m| m.open_feeder(0)),
        KeyCode::Char('O') => run(app, "open_feeder 1", |m| m.open_feeder(1)),
        KeyCode::Char('F') => run(app, "close all feeders", |m| m.close_feeders()),
        KeyCode::Char('f') => start_input(app, "feeder id (0..127):", InputTarget::Feeder),
        KeyCode::Char('1') => run(app, "conveyor_run", |m| m.conveyor_run()),
        KeyCode::Char('2') => run(app, "conveyor_stop", |m| m.conveyor_stop()),
        KeyCode::Char('3') => run(app, "conveyor_clip_pcb", |m| m.conveyor_clip_pcb()),
        KeyCode::Char('4') => run(app, "conveyor_unclip_and_move", |m| {
            m.conveyor_unclip_and_move()
        }),
        KeyCode::Char('5') => run(app, "conveyor_clear", |m| m.conveyor_clear()),
        KeyCode::Char('6') => run(app, "conveyor_check_pcb", |m| m.conveyor_check_pcb()),
        KeyCode::Char('t') => start_input(app, "track width (raw units):", InputTarget::Track),
        KeyCode::Char('T') => run(app, "track_reset", |m| m.track_reset()),
        KeyCode::Char('i') => start_input(app, "light on: id brightness", InputTarget::Light),
        KeyCode::Char('I') => run(app, "light_off", |m| m.light_off()),
        KeyCode::Char('e') => run(app, "error_light RED", |m| m.error_light(LightColor::Red)),
        KeyCode::Char('E') => run(app, "error_light YELLOW", |m| {
            m.error_light(LightColor::Yellow)
        }),
        KeyCode::Char('k') => app.log.clear(),
        KeyCode::Char('0') => {
            app.active_nozzle = NozzleId::N0;
            app.status = "active nozzle: N0".into();
        }
        KeyCode::Char('@') => {
            app.active_nozzle = NozzleId::N1;
            app.status = "active nozzle: N1".into();
        }
        KeyCode::Char('#') => {
            app.active_nozzle = NozzleId::N2;
            app.status = "active nozzle: N2".into();
        }
        KeyCode::Char('$') => {
            app.active_nozzle = NozzleId::N3;
            app.status = "active nozzle: N3".into();
        }
        _ => {}
    }
    Ok(())
}

fn handle_input_key(app: &mut App, key: KeyEvent) -> io::Result<()> {
    match key.code {
        KeyCode::Esc => cancel_input(app),
        KeyCode::Enter => submit_input(app),
        KeyCode::Backspace => {
            app.input_buf.pop();
        }
        KeyCode::Char(c) => app.input_buf.push(c),
        _ => {}
    }
    Ok(())
}

fn start_input(app: &mut App, prompt: &str, target: InputTarget) {
    app.mode = Mode::Input;
    app.input_prompt = prompt.into();
    app.input_buf.clear();
    app.input_target = Some(target);
    app.last_input_error = None;
}

fn cancel_input(app: &mut App) {
    app.mode = Mode::Menu;
    app.input_target = None;
    app.input_buf.clear();
    app.input_prompt.clear();
    app.last_input_error = None;
}

fn fail_input(app: &mut App, msg: impl Into<String>) {
    app.last_input_error = Some(msg.into());
}

fn done_input(app: &mut App) {
    app.mode = Mode::Menu;
    app.input_target = None;
    app.input_buf.clear();
    app.input_prompt.clear();
    app.last_input_error = None;
}

fn submit_input(app: &mut App) {
    let target = match app.input_target {
        Some(t) => t,
        None => {
            app.mode = Mode::Menu;
            return;
        }
    };
    let buf = app.input_buf.trim().to_string();
    let parts: Vec<&str> = buf.split_whitespace().collect();
    match target {
        InputTarget::Port => {
            if let Err(e) = open_port(app, &buf) {
                fail_input(app, format!("port: {}", e));
                return;
            }
            done_input(app);
        }
        InputTarget::MoveXy => {
            if parts.len() < 3 {
                fail_input(app, "need x y speed [type]");
                return;
            }
            let x: f64 = match parts[0].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad x");
                    return;
                }
            };
            let y: f64 = match parts[1].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad y");
                    return;
                }
            };
            let speed: u8 = match parts[2].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad speed");
                    return;
                }
            };
            let kind = match parts.get(3).map(|s| s.to_uppercase()).as_deref() {
                Some("T1") => hwcontrol::MoveType::T1,
                Some("T2") => hwcontrol::MoveType::T2,
                Some("T3") => hwcontrol::MoveType::T3,
                Some("THROW") => hwcontrol::MoveType::Throw,
                _ => hwcontrol::MoveType::Normal,
            };
            run_owned(
                app,
                format!("move_xy {} {} speed={} {:?}", x, y, speed, kind),
                move |m| m.move_xy((x, y), speed, kind),
            );
            done_input(app);
        }
        InputTarget::MoveZ => {
            if parts.len() < 2 {
                fail_input(app, "need height speed [nozzle]");
                return;
            }
            let (nozzle, off) = match parse_nozzle_opt(parts[0]) {
                Some(n) => (n, 1),
                None => (app.active_nozzle, 0),
            };
            let rest: Vec<&str> = if off == 1 {
                parts[1..].to_vec()
            } else {
                parts.clone()
            };
            if rest.len() < 2 {
                fail_input(app, "need height speed");
                return;
            }
            let height: f64 = match rest[0].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad height");
                    return;
                }
            };
            let speed: u8 = match rest[1].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad speed");
                    return;
                }
            };
            run_owned(
                app,
                format!("move_z N{:?} {} speed={}", nozzle, height, speed),
                move |m| m.move_z(nozzle, height, speed),
            );
            done_input(app);
        }
        InputTarget::Jog => {
            if parts.len() < 3 {
                fail_input(app, "need x y speed");
                return;
            }
            let x: i8 = parts[0].parse().unwrap_or(0);
            let y: i8 = parts[1].parse().unwrap_or(0);
            let speed: u8 = parts[2].parse().unwrap_or(0);
            let req = JogRequest { x, y, speed };
            run_owned(app, format!("jog_xy {:?}", req), move |m| m.jog_xy(req));
            done_input(app);
        }
        InputTarget::Rotate => {
            if parts.len() < 2 {
                fail_input(app, "need angle speed [nozzle]");
                return;
            }
            let (nozzle, off) = match parse_nozzle_opt(parts[0]) {
                Some(n) => (n, 1),
                None => (app.active_nozzle, 0),
            };
            let rest: Vec<&str> = if off == 1 {
                parts[1..].to_vec()
            } else {
                parts.clone()
            };
            if rest.len() < 2 {
                fail_input(app, "need angle speed");
                return;
            }
            let angle: f64 = match rest[0].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad angle");
                    return;
                }
            };
            let speed: u8 = match rest[1].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad speed");
                    return;
                }
            };
            run_owned(
                app,
                format!("rotate N{:?} {} speed={}", nozzle, angle, speed),
                move |m| m.rotate_nozzle(nozzle, angle, true, speed),
            );
            done_input(app);
        }
        InputTarget::Feeder => {
            let id: u8 = match buf.parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad id");
                    return;
                }
            };
            run_owned(app, format!("open_feeder {}", id), move |m| {
                m.open_feeder(id)
            });
            done_input(app);
        }
        InputTarget::Light => {
            if parts.len() < 2 {
                fail_input(app, "need id brightness");
                return;
            }
            let id: u8 = match parts[0].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad id");
                    return;
                }
            };
            let brightness: u8 = match parts[1].parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad brightness");
                    return;
                }
            };
            run_owned(app, format!("light_on {} {}", id, brightness), move |m| {
                m.light_on(id, brightness)
            });
            done_input(app);
        }
        InputTarget::Track => {
            let units: u16 = match buf.parse() {
                Ok(v) => v,
                Err(_) => {
                    fail_input(app, "bad units");
                    return;
                }
            };
            run_owned(app, format!("track_set_width {}", units), move |m| {
                m.track_set_width(units)
            });
            done_input(app);
        }
    }
}

fn parse_nozzle_opt(s: &str) -> Option<NozzleId> {
    match s {
        "0" => Some(NozzleId::N0),
        "1" => Some(NozzleId::N1),
        "2" => Some(NozzleId::N2),
        "3" => Some(NozzleId::N3),
        _ => None,
    }
}

fn open_port(app: &mut App, path: &str) -> Result<(), String> {
    let port = serialport::new(path, 115_200)
        .timeout(Duration::from_millis(500))
        .open()
        .map_err(|e| e.to_string())?;
    let transport = SerialTransport::new(port);
    let machine: OpenKayoMachine<_> = OpenKayoMachine::new(transport, MessageBuilder::default());
    app.machine = Some(Box::new(machine));
    app.status = format!("Connected to {}", path);
    app.log(format!("Connected to {}", path));
    Ok(())
}

fn connect_loopback(app: &mut App) {
    use hwcontrol::transport::LoopbackTransport;
    let transport = LoopbackTransport::new();
    let machine: OpenKayoMachine<_> = OpenKayoMachine::new(transport, MessageBuilder::default());
    app.machine = Some(Box::new(machine));
    app.status = "Loopback (frames will not be transmitted)".into();
    app.log("Loopback connected");
}

fn disconnect(app: &mut App) {
    app.machine = None;
    app.position = None;
    app.status = "Disconnected".into();
    app.log("Disconnected");
}

fn report_position(app: &mut App) {
    let Some(machine) = app.machine.as_mut() else {
        app.status = "Not connected".into();
        return;
    };
    match machine.report_position() {
        Ok(position) => {
            app.position = Some(position);
            app.status = "OK: report_position".into();
            app.log(format!(
                "OK: position X={:.3} Y={:.3} Z=[{:.3}, {:.3}, {:.3}, {:.3}]",
                position.x, position.y, position.z[0], position.z[1], position.z[2], position.z[3]
            ));
        }
        Err(e) => {
            app.status = format!("ERR: report_position ({})", e);
            app.log(format!("ERR: report_position ({})", e));
        }
    }
}

fn run<F>(app: &mut App, label: &str, mut f: F)
where
    F: FnMut(&mut dyn Machine) -> Result<(), hwcontrol::Error>,
{
    let Some(machine) = app.machine.as_mut() else {
        app.status = "Not connected".into();
        return;
    };
    match f(&mut **machine) {
        Ok(()) => {
            app.status = format!("OK: {}", label);
            app.log(format!("OK: {}", label));
        }
        Err(e) => {
            app.status = format!("ERR: {} ({})", label, e);
            app.log(format!("ERR: {} ({})", label, e));
        }
    }
}

fn run_owned<F>(app: &mut App, label: String, mut f: F)
where
    F: FnMut(&mut dyn Machine) -> Result<(), hwcontrol::Error>,
{
    let Some(machine) = app.machine.as_mut() else {
        app.status = "Not connected".into();
        return;
    };
    match f(&mut **machine) {
        Ok(()) => {
            app.status = format!("OK: {}", label);
            app.log(format!("OK: {}", label));
        }
        Err(e) => {
            app.status = format!("ERR: {} ({})", label, e);
            app.log(format!("ERR: {} ({})", label, e));
        }
    }
}

fn ui(f: &mut ratatui::Frame, app: &mut App) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Length(4),
            Constraint::Length(12),
            Constraint::Min(5),
            Constraint::Length(3),
        ])
        .split(f.area());

    let header = Paragraph::new(Line::from(vec![
        Span::styled(
            "hwcontrol ",
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        ),
        Span::raw("| nozzle "),
        Span::styled(
            format!("N{:?}", app.active_nozzle),
            Style::default()
                .fg(Color::Magenta)
                .add_modifier(Modifier::BOLD),
        ),
        Span::raw(" | "),
        Span::styled(&app.status, Style::default().fg(Color::Yellow)),
    ]))
    .block(Block::default().borders(Borders::ALL).title("Status"));
    f.render_widget(header, chunks[0]);

    let help = Paragraph::new(
        "0/@/#/$:select nozzle N0/N1/N2/N3\n\
         p:open port  l:loopback  d:disconnect\n\
         h:home all  H:home z  r:reset\n\
         s:report position\n\
         x:move XY   z:move Z (nozzle opt)   j:jog XY   R:rotate (nozzle opt)\n\
         v/V/b/B/c:vac+blow on active nozzle   C:close all\n\
         f:open feeder  F:close feeders\n\
         1..6:conveyor run/stop/clip/unclip/clear/check   t:track width   T:track reset\n\
         i:light on  I:light off  e/E:err red/yellow  k:clear log  q:quit",
    )
    .block(Block::default().borders(Borders::ALL).title("Keys"))
    .wrap(Wrap { trim: false });
    f.render_widget(help, chunks[2]);

    let position_text = match app.position {
        Some(position) => format!(
            "X: {:>9.3}  Y: {:>9.3}\nZ: [{:>7.3}, {:>7.3}, {:>7.3}, {:>7.3}]\nA-D: [{:>7.2}, {:>7.2}, {:>7.2}, {:>7.2}]",
            position.x,
            position.y,
            position.z[0],
            position.z[1],
            position.z[2],
            position.z[3],
            position.rotation[0],
            position.rotation[1],
            position.rotation[2],
            position.rotation[3],
        ),
        None => "No position report yet. Press s to wait for a position return.".into(),
    };
    let position_panel = Paragraph::new(position_text)
        .block(Block::default().borders(Borders::ALL).title("Position"))
        .style(Style::default().fg(Color::Cyan));
    f.render_widget(position_panel, chunks[1]);

    let items: Vec<ListItem> = app
        .log
        .iter()
        .rev()
        .take(chunks[3].height.saturating_sub(2) as usize)
        .map(|l| ListItem::new(l.as_str()))
        .collect();
    let log = List::new(items).block(Block::default().borders(Borders::ALL).title("Log"));
    f.render_widget(log, chunks[3]);

    let input_text = match app.mode {
        Mode::Input => {
            let err = app.last_input_error.as_deref().unwrap_or("");
            format!("{} > {}\n{}", app.input_prompt, app.input_buf, err)
        }
        Mode::Menu => "press a key...".into(),
    };
    let input = Paragraph::new(input_text)
        .block(Block::default().borders(Borders::ALL).title("Input"))
        .style(match app.mode {
            Mode::Input => Style::default().fg(Color::Green),
            Mode::Menu => Style::default().fg(Color::DarkGray),
        });
    f.render_widget(input, chunks[4]);
}
